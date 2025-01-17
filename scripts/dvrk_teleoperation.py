#!/usr/bin/env python

# Author: Junxiang Wang
# Date: 2024-04-12

# (C) Copyright 2024-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start teleoperation for (e.g.) MTML/PSM1 via:
# > rosrun dvrk_python dvrk_teleoperation -m MTML -p PSM1

import argparse
import crtk
import dvrk
from enum import Enum
import json
import math
import numpy
import PyKDL
import sys
import time

class teleoperation:
    class State(Enum):
        ALIGNING = 1
        CLUTCHED = 2
        FOLLOWING = 3

    def __init__(self, ral, master, puppet, clutch_topic, expected_interval, operator_present_topic = ""):
        print('Initialzing dvrk_teleoperation for {} and {}'.format(master.name(), puppet.name()))
        self.ral = ral
        self.expected_interval = expected_interval

        self.master = master
        self.puppet = puppet

        self.scale = 0.2

        self.gripper_max = 60 * math.pi / 180
        self.gripper_zero = 0.0 # Set to e.g. 20 degrees if gripper cannot close past zero
        self.jaw_min = -20 * math.pi / 180
        self.jaw_max = 80 * math.pi / 180
        self.jaw_rate = 2 * math.pi

        self.operator_orientation_tolerance = 5 * math.pi / 180
        self.operator_gripper_threshold = 5 * math.pi / 180
        self.operator_roll_threshold = 3 * math.pi / 180

        self.gripper_to_jaw_scale = self.jaw_max / (self.gripper_max - self.gripper_zero)
        self.gripper_to_jaw_offset = -self.gripper_zero * self.gripper_to_jaw_scale

        self.operator_is_active = False
        if operator_present_topic != "":
            self.operator_is_present = False
            self.operator_button = crtk.joystick_button(ral, operator_present_topic)
            self.operator_button.set_callback(self.on_operator_present)
        else:
            self.operator_is_present = True # if not given, then always assume present

        self.clutch_pressed = False
        self.clutch_button = crtk.joystick_button(ral, clutch_topic)
        self.clutch_button.set_callback(self.on_clutch)

    def on_operator_present(self, present):
        self.operator_is_present = present
        if not present:
            self.operator_is_active = False

    def on_clutch(self, clutch_pressed):
        self.clutch_pressed = clutch_pressed

    def alignment_offset(self):
        return self.master.measured_cp().M.Inverse() * self.puppet.setpoint_cp().M

    def update_initial_state(self):
        self.master_cartesian_initial = self.master.measured_cp()
        self.puppet_cartesian_initial = self.puppet.setpoint_cp()
        self.alignment_offset_initial = self.alignment_offset()

    def gripper_to_jaw(self, gripper_angle):
        jaw_angle = self.gripper_to_jaw_scale * gripper_angle + self.gripper_to_jaw_offset

        # make sure we don't set goal past joint limits
        return max(jaw_angle, self.jaw_min)

    def jaw_to_gripper(self, jaw_angle):
        return (jaw_angle - self.gripper_to_jaw_offset) / self.gripper_to_jaw_scale

    def check_arm_state(self):
        if not self.puppet.is_homed():
            print(f'ERROR: {self.ral.node_name()}: puppet ({self.puppet.name()}) is not homed anymore')
            self.running = False
        if not self.master.is_homed():
            print(f'ERROR: {self.ral.node_name()}: master ({self.master.name()}) is not homed anymore')
            self.running = False

    def enter_aligning(self):
        self.current_state = teleoperation.State.ALIGNING
        self.last_align = None
        self.last_operator_prompt = time.perf_counter()

        self.master.use_gravity_compensation(True)
        self.puppet.hold()

        self.operator_roll_min = math.pi * 100
        self.operator_roll_max = -math.pi * 100
        self.operator_gripper_min = math.pi * 100
        self.operator_gripper_max = -math.pi * 100

    def transition_aligning(self):
        if self.operator_is_active and self.clutch_pressed:
            self.enter_clutched()
            return

        orientation_error, _ = self.alignment_offset().GetRotAngle()
        aligned = orientation_error <= self.operator_orientation_tolerance
        if aligned and self.operator_is_active:
            self.enter_following()

    def run_aligning(self):
        orientation_error, _ = self.alignment_offset().GetRotAngle()

        # if operator is inactive, use gripper or roll to detect when the user is ready
        if self.operator_is_present:
            gripper = self.master.gripper.measured_js()[0][0]
            self.operator_gripper_max = max(gripper, self.operator_gripper_max)
            self.operator_gripper_min = min(gripper, self.operator_gripper_min)
            gripper_range = self.operator_gripper_max - self.operator_gripper_min
            if gripper_range >= self.operator_gripper_threshold:
                self.operator_is_active = True

            master_rotation, puppet_rotation = self.master.measured_cp().M, self.puppet.setpoint_cp().M
            master_y_axis = PyKDL.Vector(master_rotation[0,1], master_rotation[1,1], master_rotation[2,1])
            puppet_y_axis = PyKDL.Vector(puppet_rotation[0,1], puppet_rotation[1,1], puppet_rotation[2,1])
            roll = math.acos(PyKDL.dot(puppet_y_axis, master_y_axis))

            self.operator_roll_max = max(roll, self.operator_roll_max)
            self.operator_roll_min = min(roll, self.operator_roll_min)
            roll_range = self.operator_roll_max - self.operator_roll_min
            if roll_range >= self.operator_roll_threshold:
                self.operator_is_active = True

        # periodically send move_cp to MTM to align with PSM
        aligned = orientation_error <= self.operator_orientation_tolerance
        now = time.perf_counter()
        if not self.last_align or now - self.last_align > 4.0:
            move_cp = PyKDL.Frame(self.puppet.setpoint_cp().M, self.master.setpoint_cp().p)
            self.master.move_cp(move_cp)
            self.last_align = now

        # periodically notify operator if un-aligned or operator is inactive
        if self.operator_is_present and now - self.last_operator_prompt > 4.0:
            self.last_operator_prompt = now
            if not aligned:
                print(f'Unable to align master ({self.master.name()}), angle error is {orientation_error * 180 / math.pi} (deg)')
            elif not self.operator_is_active:
                print(f'To begin teleop, pinch/twist master ({self.master.name()}) gripper a bit')

    def enter_clutched(self):
        self.current_state = teleoperation.State.CLUTCHED

        # let MTM position move freely, but lock orientation
        wrench = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.master.body.servo_cf(wrench)
        self.master.lock_orientation(self.master.measured_cp().M)

        self.puppet.hold()

    def transition_clutched(self):
        if not self.clutch_pressed or not self.operator_is_present:
            self.enter_aligning()

    def run_clutched(self):
        pass

    def enter_following(self):
        self.current_state = teleoperation.State.FOLLOWING
        # update MTM/PSM origins position
        self.update_initial_state()

        # set up gripper ghost to rate-limit jaw speed
        jaw_setpoint = self.puppet.jaw.setpoint_js()[0]
        if len(jaw_setpoint) != 1:
            print(f'{self.ral.node_name()}: unable to get jaw position. Make sure there is an instrument on the puppet ({self.puppet.name()})')
            self.running = False
        self.gripper_ghost = self.jaw_to_gripper(jaw_setpoint[0])

        self.master.use_gravity_compensation(True)

    def transition_following(self):
        if not self.operator_is_present:
            self.enter_aligning()
        elif self.clutch_pressed:
            self.enter_clutched()

    def run_following(self):
        # let arm move freely
        wrench = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.master.body.servo_cf(wrench)

        ### Cartesian pose teleop
        master_position = self.master.measured_cp()

        # translation
        master_translation = master_position.p - self.master_cartesian_initial.p
        puppet_translation = master_translation * self.scale
        puppet_translation = puppet_translation + self.puppet_cartesian_initial.p

        # rotation
        puppet_rotation = master_position.M * self.alignment_offset_initial

        puppet_cartesian_goal = PyKDL.Frame(puppet_rotation, puppet_translation)
        self.puppet.servo_cp(puppet_cartesian_goal)

        ### Jaw/gripper teleop
        gripper_measured_js = self.master.gripper.measured_js()
        current_gripper = gripper_measured_js[0][0]

        ghost_lag = current_gripper - self.gripper_ghost
        max_delta = self.jaw_rate * self.expected_interval
        # move ghost at most max_delta towards current gripper
        self.gripper_ghost += math.copysign(min(abs(ghost_lag), max_delta), ghost_lag)
        self.puppet.jaw.servo_jp(numpy.array([self.gripper_to_jaw(self.gripper_ghost)]))

    def home(self):
        print("Homing arms...")
        timeout = 10.0 # seconds
        if not self.puppet.enable(timeout) or not self.puppet.home(timeout):
            print('    ! failed to home {} within {} seconds'.format(self.puppet.name(), timeout))
            return False

        if not self.master.enable(timeout) or not self.master.home(timeout):
            print('    ! failed to home {} within {} seconds'.format(self.master.name(), timeout))
            return False

        print("    Homing is complete")
        return True

    def run(self):
        homed_successfully = self.home()
        if not homed_successfully:
            return

        teleop_rate = self.ral.create_rate(int(1/self.expected_interval))
        print("Running teleop at {} Hz".format(int(1/self.expected_interval)))

        self.enter_aligning()
        self.running = True

        while not self.ral.is_shutdown():
            # check if teleop state should transition
            if self.current_state == teleoperation.State.ALIGNING:
                self.transition_aligning()
            elif self.current_state == teleoperation.State.CLUTCHED:
                self.transition_clutched()
            elif self.current_state == teleoperation.State.FOLLOWING:
                self.transition_following()
            else:
                raise RuntimeError("Invalid state: {}".format(self.current_state))

            self.check_arm_state()
            if not self.running:
                break

            # run teleop state handler
            if self.current_state == teleoperation.State.ALIGNING:
                self.run_aligning()
            elif self.current_state == teleoperation.State.CLUTCHED:
                self.run_clutched()
            elif self.current_state == teleoperation.State.FOLLOWING:
                self.run_following()
            else:
                raise RuntimeError("Invalid state: {}".format(self.current_state))

            teleop_rate.sleep()


if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser(description = __doc__,
                                     formatter_class = argparse.RawTextHelpFormatter)
    parser.add_argument('-m', '--mtm', type = str, default='MTML', # required = True,
                        choices = ['MTML', 'MTMR'],
                        help = 'MTM arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-p', '--psm', type = str, default='PSM2', # required = True,
                        choices = ['PSM1', 'PSM2', 'PSM3'],
                        help = 'PSM arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-c', '--clutch', type = str, default='/footpedals/clutch',
                        help = 'ROS topic corresponding to clutch button/pedal input')
    parser.add_argument('-o', '--operator', type = str, default='/footpedals/coag',
                        help = 'ROS topic corresponding to operator present button/pedal/sensor input')
    parser.add_argument('-i', '--interval', type=float, default=0.005,
                        help = 'expected interval in seconds between messages sent by the device')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_python_teleoperation')
    mtm = dvrk.mtm(ral, args.mtm, args.interval*2)
    psm = dvrk.psm(ral, args.psm, args.interval*2)
    application = teleoperation(ral, mtm, psm, args.clutch, args.interval, operator_present_topic=args.operator)
    ral.spin_and_execute(application.run)
