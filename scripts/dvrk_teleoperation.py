#!/usr/bin/env python

# Author: Junxiang Wang
# Date: 2024-04-12

# (C) Copyright 2024-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

### Instructions:
# First start a console:
# > ros2 run dvrk_robot dvrk_console_json -j jhu-dVRK/console-MTML-PSM2-Teleop.json
# or to use a Falon/ForceDimension haptic device as the MTM:
# > ros2 run dvrk_robot dvrk_console_json -j jhu-dVRK/console-Falcon-PSM2-TeleopDerived.json -C -p 0.001

# Next, start the teleoperation script for (e.g.) MTML/PSM1 via:
# > rosrun dvrk_python dvrk_teleoperation -m MTML -p PSM1
# If using a haptic device with unactuated wrist, make sure to add the -n flag

import argparse
import crtk
from enum import Enum
import geometry_msgs.msg
import math
import numpy
import PyKDL
import std_msgs.msg
import sys
import time

class teleoperation:
    class State(Enum):
        ALIGNING = 1
        CLUTCHED = 2
        FOLLOWING = 3

    def __init__(self, ral, master, puppet, clutch_topic, run_period, align_mtm, operator_present_topic = ""):
        print('Initialzing dvrk_teleoperation for {} and {}'.format(master.name, puppet.name))
        self.ral = ral
        self.run_period = run_period

        self.master = master
        self.puppet = puppet

        self.scale = 0.2

        self.gripper_max = 60 * math.pi / 180
        self.gripper_zero = 0.0 # Set to e.g. 20 degrees if gripper cannot close past zero
        self.jaw_min = -20 * math.pi / 180
        self.jaw_max = 80 * math.pi / 180
        self.jaw_rate = 2 * math.pi

        self.can_align_mtm = align_mtm

        # slowly eliminate alignment offset if we can align mtm,
        # otherwise maintain fixed initial alignment offset
        self.align_rate = 0.25 * math.pi if self.can_align_mtm else 0.0

        # don't require alignment before beginning teleop if mtm wrist can't be actuated
        self.operator_orientation_tolerance = 5 * math.pi / 180 if self.can_align_mtm else math.pi
        self.operator_gripper_threshold = 5 * math.pi / 180
        self.operator_roll_threshold = 3 * math.pi / 180

        self.gripper_to_jaw_scale = self.jaw_max / (self.gripper_max - self.gripper_zero)
        self.gripper_to_jaw_offset = -self.gripper_zero * self.gripper_to_jaw_scale

        self.operator_is_active = False
        if operator_present_topic:
            self.operator_is_present = False
            self.operator_button = crtk.joystick_button(ral, operator_present_topic)
            self.operator_button.set_callback(self.on_operator_present)
        else:
            self.operator_is_present = True # if not given, then always assume present

        self.clutch_pressed = False
        self.clutch_button = crtk.joystick_button(ral, clutch_topic)
        self.clutch_button.set_callback(self.on_clutch)

    # callback for operator pedal/button
    def on_operator_present(self, present):
        self.operator_is_present = present
        if not present:
            self.operator_is_active = False

    # callback for clutch pedal/button
    def on_clutch(self, clutch_pressed):
        self.clutch_pressed = clutch_pressed

    # compute relative orientation of mtm and psm
    def alignment_offset(self):
        return self.master.measured_cp()[0].M.Inverse() * self.puppet.setpoint_cp()[0].M

    # set relative origins for clutching and alignment offset
    def update_initial_state(self):
        self.master_cartesian_initial = self.master.measured_cp()[0]
        self.puppet_cartesian_initial = self.puppet.setpoint_cp()[0]
        self.alignment_offset_initial = self.alignment_offset()
        self.offset_angle, self.offset_axis = self.alignment_offset_initial.GetRotAngle()

    def gripper_to_jaw(self, gripper_angle):
        jaw_angle = self.gripper_to_jaw_scale * gripper_angle + self.gripper_to_jaw_offset

        # make sure we don't set goal past joint limits
        return max(jaw_angle, self.jaw_min)

    def jaw_to_gripper(self, jaw_angle):
        return (jaw_angle - self.gripper_to_jaw_offset) / self.gripper_to_jaw_scale

    def check_arm_state(self):
        if not self.puppet.is_homed():
            print(f'ERROR: {self.ral.node_name()}: puppet ({self.puppet.name}) is not homed anymore')
            self.running = False
        if not self.master.is_homed():
            print(f'ERROR: {self.ral.node_name()}: master ({self.master.name}) is not homed anymore')
            self.running = False

    def enter_aligning(self):
        self.current_state = teleoperation.State.ALIGNING
        self.last_align = None
        self.last_operator_prompt = time.perf_counter()

        self.master.use_gravity_compensation(True)
        self.puppet.hold()

        # reset operator activity data in case operator is inactive
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

        # if operator is inactive, use gripper or roll activity to detect when the user is ready
        if self.operator_is_present:
            gripper = self.master.gripper.measured_js()[0][0]
            self.operator_gripper_max = max(gripper, self.operator_gripper_max)
            self.operator_gripper_min = min(gripper, self.operator_gripper_min)
            gripper_range = self.operator_gripper_max - self.operator_gripper_min
            if gripper_range >= self.operator_gripper_threshold:
                self.operator_is_active = True

            # determine amount of roll around z axis by rotation of y-axis
            master_rotation, puppet_rotation = self.master.measured_cp()[0].M, self.puppet.setpoint_cp()[0].M
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
            move_cp = PyKDL.Frame(self.puppet.setpoint_cp()[0].M, self.master.setpoint_cp()[0].p)
            self.master.move_cp(move_cp)
            self.last_align = now

        # periodically notify operator if un-aligned or operator is inactive
        if self.operator_is_present and now - self.last_operator_prompt > 4.0:
            self.last_operator_prompt = now
            if not aligned:
                print(f'Unable to align master ({self.master.name}), angle error is {orientation_error * 180 / math.pi} (deg)')
            elif not self.operator_is_active:
                print(f'To begin teleop, pinch/twist master ({self.master.name}) gripper a bit')

    def enter_clutched(self):
        self.current_state = teleoperation.State.CLUTCHED

        # let MTM position move freely, but lock orientation
        wrench = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.master.body.servo_cf(wrench)
        self.master.lock_orientation(self.master.measured_cp()[0].M)

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
            print(f'{self.ral.node_name()}: unable to get jaw position. Make sure there is an instrument on the puppet ({self.puppet.name})')
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
        master_position = self.master.measured_cp()[0]

        # translation
        master_translation = master_position.p - self.master_cartesian_initial.p
        puppet_translation = master_translation * self.scale
        puppet_translation = puppet_translation + self.puppet_cartesian_initial.p

        # set rotation of psm to match mtm plus alignment offset
        # if we can actuate the MTM, we slowly reduce the alignment offset to zero over time
        max_delta = self.align_rate * self.run_period
        self.offset_angle += math.copysign(min(abs(self.offset_angle), max_delta), -self.offset_angle)
        alignment_offset = PyKDL.Rotation.Rot(self.offset_axis, self.offset_angle)
        puppet_rotation = master_position.M * alignment_offset

        puppet_cartesian_goal = PyKDL.Frame(puppet_rotation, puppet_translation)
        self.puppet.servo_cp(puppet_cartesian_goal)

        ### Jaw/gripper teleop
        gripper_measured_js = self.master.gripper.measured_js()
        current_gripper = gripper_measured_js[0][0]

        ghost_lag = current_gripper - self.gripper_ghost
        max_delta = self.jaw_rate * self.run_period
        # move ghost at most max_delta towards current gripper
        self.gripper_ghost += math.copysign(min(abs(ghost_lag), max_delta), ghost_lag)
        self.puppet.jaw.servo_jp(numpy.array([self.gripper_to_jaw(self.gripper_ghost)]))

    def home(self):
        print("Homing arms...")
        timeout = 10.0 # seconds
        if not self.puppet.enable(timeout) or not self.puppet.home(timeout):
            print('    ! failed to home {} within {} seconds'.format(self.puppet.name, timeout))
            return False

        if not self.master.enable(timeout) or not self.master.home(timeout):
            print('    ! failed to home {} within {} seconds'.format(self.master.name, timeout))
            return False

        print("    Homing is complete")
        return True

    def run(self):
        homed_successfully = self.home()
        if not homed_successfully:
            return

        teleop_rate = self.ral.create_rate(int(1/self.run_period))
        print("Running teleop at {} Hz".format(int(1/self.run_period)))

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

class MTM:
    class ServoCF:
        def __init__(self, ral, timeout):
            self.utils = crtk.utils(self, ral, timeout)
            self.utils.add_servo_cf()

    class Gripper:
        def __init__(self, ral, timeout):
            self.utils = crtk.utils(self, ral, timeout)
            self.utils.add_measured_js()

    def __init__(self, ral, arm_name, timeout):
        self.name = arm_name
        self.ral = ral.create_child(arm_name)
        self.utils = crtk.utils(self, self.ral, timeout)

        self.utils.add_operating_state()
        self.utils.add_measured_cp()
        self.utils.add_setpoint_cp()
        self.utils.add_move_cp()

        self.gripper = self.Gripper(self.ral.create_child('gripper'), timeout)
        self.body = self.ServoCF(self.ral.create_child('body'), timeout)

        # non-CRTK topics
        self.lock_orientation_pub = self.ral.publisher('lock_orientation',
                                                        geometry_msgs.msg.Quaternion,
                                                        latch = True, queue_size = 1)
        self.unlock_orientation_pub = self.ral.publisher('unlock_orientation',
                                                         std_msgs.msg.Empty,
                                                         latch = True, queue_size = 1)
        self.use_gravity_compensation_pub = self.ral.publisher('use_gravity_compensation',
                                                                std_msgs.msg.Bool,
                                                                latch = True, queue_size = 1)

    def lock_orientation(self, orientation):
        """orientation should be a PyKDL.Rotation object"""
        q = geometry_msgs.msg.Quaternion()
        q.x, q.y, q.z, q.w = orientation.GetQuaternion()
        self.lock_orientation_pub.publish(q)

    def unlock_orientation(self):
        self.unlock_orientation_pub.publish(std_msgs.msg.Empty())

    def use_gravity_compensation(self, gravity_compensation):
        """Turn on/off gravity compensation (only applies to Cartesian effort mode)"""
        msg = std_msgs.msg.Bool(data=gravity_compensation)
        self.use_gravity_compensation_pub.publish(msg)

class PSM:
    class Jaw:
        def __init__(self, ral, timeout):
            self.utils = crtk.utils(self, ral, timeout)
            self.utils.add_setpoint_js()
            self.utils.add_servo_jp()

    def __init__(self, ral, arm_name, timeout):
        self.name = arm_name
        self.ral = ral.create_child(arm_name)
        self.utils = crtk.utils(self, self.ral, timeout)

        self.utils.add_operating_state()
        self.utils.add_setpoint_cp()
        self.utils.add_servo_cp()
        self.utils.add_hold()

        self.jaw = self.Jaw(self.ral.create_child('jaw'), timeout)

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
    parser.add_argument('-o', '--operator', type = str, default='/footpedals/coag', const=None, nargs='?',
                        help = 'ROS topic corresponding to operator present button/pedal/sensor input')
    parser.add_argument('-n', '--no-mtm-alignment', action='store_true',
                        help="don't align mtm (useful for using haptic devices as MTM which don't have wrist actuation)")
    parser.add_argument('-i', '--interval', type=float, default=0.005,
                        help = 'time interval/period to run at - should be longer than console\'s period to prevent timeouts')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_python_teleoperation')
    mtm = MTM(ral, args.mtm, 4*args.interval)
    psm = PSM(ral, args.psm, 4*args.interval)
    application = teleoperation(ral, mtm, psm, args.clutch, args.interval,
                                not args.no_mtm_alignment, operator_present_topic=args.operator)
    ral.spin_and_execute(application.run)
