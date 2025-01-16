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
import crtk_msgs.msg
from enum import Enum
import geometry_msgs.msg
import json
import math
import numpy
import os
import PyKDL
import std_msgs.msg
import sys
import time

class dvrk_teleoperation:
    class state(Enum):
        ALIGNING = 1
        CLUTCHED = 2
        FOLLOWING = 3

    def __init__(self, ral, master, puppet, clutch_topic, expected_interval, operator_present_topic = "", config_file_name = ""):
        print('Initialzing dvrk_teleoperation for {} and {}'.format(master.name(), puppet.name()))
        self.ral = ral
        self.expected_interval = expected_interval

        self.master = master
        # Required features of master
        getattr(self.master, "measured_cp")
        getattr(self.master, "setpoint_cp")
        getattr(self.master, "move_cp")
        getattr(self.master, "use_gravity_compensation")
        getattr(self.master, "operating_state")
        getattr(self.master, "state_command")

        self.puppet = puppet
        # Required features of puppet
        getattr(self.puppet, "setpoint_cp")
        getattr(self.puppet, "servo_cp")
        getattr(self.puppet, "hold")
        getattr(self.puppet, "operating_state")
        getattr(self.puppet, "state_command")

        self.jaw_caught_up_after_clutch = False
        self.scale = 0.2
        self.jaw_ignore = False
        self.jaw_rate = 2 * math.pi
        self.jaw_rate_back_from_clutch = 0.05 * math.pi
        self.gripper_max = 60 * math.pi / 180
        self.gripper_zero = 0
        self.jaw_min = -20 * math.pi / 180
        self.jaw_max = 80 * math.pi / 180
        self.operator_orientation_tolerance = 5 * math.pi / 180
        self.operator_gripper_threshold = 5 * math.pi / 180
        self.operator_roll_threshold = 3 * math.pi / 180
        self.tolerance_back_from_clutch = 2 * math.pi / 180

        self.puppet_jaw_servo_jp = numpy.zeros((6,))

        self.operator_is_active = False

        if operator_present_topic != "":
            self.operator_is_present = False
            self.operator_button = crtk.joystick_button(ral, operator_present_topic)
            self.operator_button.set_callback(self.on_operator_present)
        else:
            self.operator_is_present = True # if not given, then always assume present

        self.current_state = None

        self.clutch_pressed = False
        self.clutch_button = crtk.joystick_button(ral, clutch_topic)
        self.clutch_button.set_callback(self.on_clutch)

        self.check_jaw()

    def check_jaw(self):
        # check if functions for jaw are connected
        if not self.jaw_ignore:
            if not callable(getattr(self.puppet.jaw, "setpoint_js", None)) or not callable(getattr(self.puppet.jaw, "servo_jp", None)):
                print(f'{self.ral.node_name()}: optional functions \"jaw/servo_jp\" and \"jaw/setpoint_js\" are not connected, setting \"ignore-jaw\" to true')
                self.jaw_ignore = True

    def on_operator_present(self, present):
        self.operator_is_present = present
        if not present:
            self.operator_is_active = False

    def on_clutch(self, clutch_pressed):
        self.clutch_pressed = clutch_pressed

    def update_align_offset(self):
        desired_orientation = self.puppet.setpoint_cp().M
        self.alignment_offset = self.master.measured_cp().M.Inverse() * desired_orientation
        return desired_orientation

    def update_initial_state(self):
        self.master_cartesian_initial = self.master.measured_cp()
        self.puppet_cartesian_initial = self.puppet.setpoint_cp()
        self.update_align_offset()
        self.alignment_offset_initial = self.alignment_offset
        # TODO: missing base frame (?) here

    def gripper_to_jaw(self, gripper_angle):
        return self.gripper_to_jaw_scale * gripper_angle + self.gripper_to_jaw_offset

    def jaw_to_gripper(self, jaw_angle):
        return (jaw_angle - self.gripper_to_jaw_offset) / self.gripper_to_jaw_scale

    def update_gripper_to_jaw_configuration(self):
        # jaw_min = self.gripper_zero
        # jaw_max = self.gripper_max

        self.gripper_to_jaw_position_min = self.jaw_min
        # TODO: add configuration_js? -- not really

        self.gripper_to_jaw_scale = self.jaw_max / (self.gripper_max - self.gripper_zero)
        self.gripper_to_jaw_offset = -self.gripper_zero / self.gripper_to_jaw_scale

    def run_all_states(self):
        if not self.puppet.is_homed():
            print(f'ERROR: {self.ral.node_name()}: puppet ({self.puppet.name()}) is not in state \"ENABLED\" anymore')
            self.running = False
        if not self.master.is_homed():
            print(f'ERROR: {self.ral.node_name()}: puppet ({self.master.name()}) is not in state \"READY\" anymore')
            self.running = False

    def enter_aligning(self):
        self.current_state = self.state.ALIGNING
        self.last_align = None

        self.master.use_gravity_compensation(True)
        self.puppet.hold()

        if not self.jaw_ignore:
            self.update_gripper_to_jaw_configuration()

        self.operator_roll_min = math.pi * 100
        self.operator_roll_max = -math.pi * 100
        self.operator_gripper_min = math.pi * 100
        self.operator_gripper_max = -math.pi * 100

    def transition_aligning(self):
        if self.operator_is_present and self.clutch_pressed:
            self.enter_clutched()
            return

        self.update_align_offset()
        orientation_error, _ = self.alignment_offset.GetRotAngle()
        aligned = orientation_error <= self.operator_orientation_tolerance
        if aligned and self.operator_is_present and self.operator_is_active:
            self.enter_following()

    def run_aligning(self):
        desired_orientation = self.update_align_offset()
        # set error to align MTM to PSM
        orientation_error, _ = self.alignment_offset.GetRotAngle()

        # if not active, use gripper and/or roll to detect if the user is ready
        if not self.operator_is_active:
            gripper_range = 0
            if callable(getattr(self.master.gripper, "measured_js", None)):
                self.master_gripper_measured_js = self.master.gripper.measured_js()
                gripper = self.master_gripper_measured_js[0][0]
                if gripper > self.operator_gripper_max:
                    self.operator_gripper_max = gripper
                elif gripper < self.operator_gripper_min:
                    self.operator_gripper_min = gripper
                gripper_range = self.operator_gripper_max - self.operator_gripper_min

            # checking roll
            measured_cp = self.master.measured_cp()
            roll = math.acos(PyKDL.dot(PyKDL.Vector(desired_orientation[0,1], desired_orientation[1,1], desired_orientation[2,1]),
                                       PyKDL.Vector(measured_cp.M[0,1], measured_cp.M[1,1], measured_cp.M[2,1])))
            if roll > self.operator_roll_max:
                self.operator_roll_max = roll
            elif roll < self.operator_gripper_min:
                self.operator_roll_min = roll
            roll_range = self.operator_roll_max - self.operator_roll_min

            if gripper_range >= self.operator_gripper_threshold:
                self.operator_is_active = True
                print(f'Made active by gripper: {gripper_range}')
            elif roll_range >= self.operator_roll_threshold:
                self.operator_is_active = True
                print(f'Made active by roll: {roll_range}')
            elif gripper_range + roll_range > 0.8 * (self.operator_gripper_threshold + self.operator_roll_threshold):
                self.operator_is_active = True
                print(f'Made active by combination: {gripper_range} + {roll_range}')

        aligned = orientation_error <= self.operator_orientation_tolerance
        if not self.last_align or time.perf_counter() - self.last_align > 4.0:
            move_cp = PyKDL.Frame(self.puppet.setpoint_cp().M, self.master.setpoint_cp().p)
            self.master.move_cp(move_cp)
            self.last_align = time.perf_counter()

        if self.operator_is_present and self.last_align and time.perf_counter() - self.last_align > 4.0:
            if not aligned:
                print(f'Unable to align master ({self.master.name()}), angle error is {orientation_error * 180 / math.pi} (deg)')
            elif not self.operator_is_active:
                print(f'Pinch/twist master ({self.master.name()}) gripper a bit')

    def enter_clutched(self):
        self.current_state = self.state.CLUTCHED

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
        self.current_state = self.state.FOLLOWING
        # update MTM/PSM origins position
        self.update_initial_state()

        # set gripper ghost if needed
        if not self.jaw_ignore:
            self.jaw_caught_up_after_clutch = False
            # gripper ghost
            self.puppet_jaw_setpoint_js = self.puppet.jaw.setpoint_js()
            if len(self.puppet_jaw_setpoint_js[0]) != 1:
                print(f'{self.ral.node_name()}: unable to get jaw position. Make sure there is an instrument on the puppet ({self.puppet.name()})')
                self.running = False
            current_jaw = self.puppet_jaw_setpoint_js[0][0]
            self.gripper_ghost = self.jaw_to_gripper(current_jaw)

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

        master_position = self.master.measured_cp()

        # translation
        master_translation = master_position.p - self.master_cartesian_initial.p
        puppet_translation = master_translation * self.scale
        puppet_translation = puppet_translation + self.puppet_cartesian_initial.p

        # rotation
        puppet_rotation = master_position.M * self.alignment_offset_initial

        puppet_cartesian_goal = PyKDL.Frame(puppet_rotation, puppet_translation)
        self.puppet.servo_cp(puppet_cartesian_goal)

        if not self.jaw_ignore:
            if callable(getattr(self.master.gripper, "measured_js", None)):
                self.master_gripper_measured_js = self.master.gripper.measured_js(age=0.05)
                current_gripper = self.master_gripper_measured_js[0][0]
                # see if we caught up
                if not self.jaw_caught_up_after_clutch:
                    error = abs(current_gripper - self.gripper_ghost)
                    if error < self.tolerance_back_from_clutch:
                        self.jaw_caught_up_after_clutch = True
                # pick rate based on back from clutch or not
                jaw_rate = self.jaw_rate if self.jaw_caught_up_after_clutch else self.jaw_rate_back_from_clutch
                delta = jaw_rate * self.expected_interval

                if self.gripper_ghost <= (current_gripper - delta):
                    self.gripper_ghost += delta
                elif self.gripper_ghost >= (current_gripper + delta):
                    self.gripper_ghost -= delta

                self.puppet_jaw_servo_jp[0] = self.gripper_to_jaw(self.gripper_ghost)
                # make sure we don't set goal past joint limits
                if self.puppet_jaw_servo_jp[0] < self.gripper_to_jaw_position_min:
                    self.puppet_jaw_servo_jp[0] = self.gripper_to_jaw_position_min
                    self.gripper_ghost = self.jaw_to_gripper(self.gripper_to_jaw_position_min)
                self.puppet.jaw.servo_jp(self.puppet_jaw_servo_jp)
            else:
                self.puppet_jaw_servo_jp[0] = 45 * math.pi / 180
                self.puppet.jaw.servo_jp(self.puppet_jaw_servo_jp)

    def home(self):
        print("Homing arms...")
        timeout = 10.0 # seconds
        if not self.puppet.enable(timeout):
            print('    ! failed to enable {} within {} seconds'.format(self.puppet.name(), timeout))
            return False

        if not self.master.enable(timeout):
            print('    ! failed to enable {} within {} seconds'.format(self.master.name(), timeout))
            return False

        if not self.puppet.home(timeout):
            print('    ! failed to home {} within {} seconds'.format(self.puppet.name(), timeout))
            return False

        if not self.master.home(timeout):
            print('    ! failed to home {} within {} seconds'.format(self.master.name(), timeout))
            return False

        print("    Homing is complete")
        return True

    def run(self):
        homed_successfully = self.home()
        if not homed_successfully:
            return

        teleop_rate = self.ral.create_rate(int(1/self.expected_interval))

        self.enter_aligning()
        self.running = True

        while not self.ral.is_shutdown():
            if self.current_state == self.state.ALIGNING:
                self.transition_aligning()
            elif self.current_state == self.state.CLUTCHED:
                self.transition_clutched()
            elif self.current_state == self.state.FOLLOWING:
                self.transition_following()
            else:
                raise RuntimeError("Invalid state: {}".format(self.current_state))

            if not self.running:
                break

            if self.current_state == self.state.ALIGNING:
                self.run_aligning()
            elif self.current_state == self.state.CLUTCHED:
                self.run_clutched()
            elif self.current_state == self.state.FOLLOWING:
                self.run_following()
            else:
                raise RuntimeError("Invalid state: {}".format(self.current_state))

            teleop_rate.sleep()

class mtm_teleop(object):
    class __ServoCf:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_servo_cf()

    class __Gripper:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_measured_js()

    def __init__(self, ral, arm_name, expected_interval = 0.01):
        """Requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `PSM1`"""
        self.__name = arm_name
        self.__ral = ral.create_child(arm_name)

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_measured_cp()
        self.__crtk_utils.add_setpoint_cp()
        self.__crtk_utils.add_move_cp()

        self.gripper = self.__Gripper(self.__ral.create_child('/gripper'), expected_interval)
        self.body = self.__ServoCf(self.__ral.create_child('body'), expected_interval)

        # publishers
        self.__lock_orientation_publisher = self.__ral.publisher('lock_orientation',
                                                                 geometry_msgs.msg.Quaternion,
                                                                 latch = True, queue_size = 1)
        self.__unlock_orientation_publisher = self.__ral.publisher('unlock_orientation',
                                                                   std_msgs.msg.Empty,
                                                                   latch = True, queue_size = 1)
        self.__use_gravity_compensation_pub = self.__ral.publisher('/use_gravity_compensation',
                                                                   std_msgs.msg.Bool,
                                                                   latch = True, queue_size = 1)

    def name(self):
        return self.__name

    def lock_orientation_as_is(self):
        "Lock orientation based on current orientation"
        current = self.setpoint_cp()
        self.lock_orientation(current.M)

    def lock_orientation(self, orientation):
        """Lock orientation, expects a PyKDL rotation matrix (PyKDL.Rotation)"""
        q = geometry_msgs.msg.Quaternion()
        q.x, q.y, q.z, q.w = orientation.GetQuaternion()
        self.__lock_orientation_publisher.publish(q)

    def unlock_orientation(self):
        "Unlock orientation"
        e = std_msgs.msg.Empty()
        self.__unlock_orientation_publisher.publish(e)

    def use_gravity_compensation(self, gravity_compensation):
        """Turn on/off gravity compensation in cartesian effort mode"""
        g = std_msgs.msg.Bool()
        g.data = gravity_compensation
        self.__use_gravity_compensation_pub.publish(g)


class psm_teleop(object):
    class __Jaw:
        def __init__(self, ral, expected_interval):
            self.__crtk_utils = crtk.utils(self, ral, expected_interval)
            self.__crtk_utils.add_setpoint_js()
            self.__crtk_utils.add_servo_jp()

    def __init__(self, ral, arm_name, expected_interval = 0.01):
        """Requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `PSM1`"""
        self.__name = arm_name
        self.__ral = ral.create_child(arm_name)

        # crtk features
        self.__crtk_utils = crtk.utils(self, self.__ral, expected_interval)

        self.__crtk_utils.add_operating_state()
        self.__crtk_utils.add_setpoint_cp()
        self.__crtk_utils.add_servo_cp()
        self.__crtk_utils.add_hold()

        self.jaw = self.__Jaw(self.__ral.create_child('/jaw'), expected_interval)

    def name(self):
        return self.__name


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
    mtm = mtm_teleop(ral, args.mtm, args.interval*2)
    psm = psm_teleop(ral, args.psm, args.interval*2)
    application = dvrk_teleoperation(ral, mtm, psm, args.clutch, args.interval, operator_present_topic=args.operator, config_file_name="")
    ral.spin_and_execute(application.run)
