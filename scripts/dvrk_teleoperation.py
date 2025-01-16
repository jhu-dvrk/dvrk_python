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
        DISABLED = 1
        SETTING_ARMS_STATE = 2
        ALIGNING_ARM = 3
        ENABLED = 4

    class operator_present_button(crtk.joystick_button):
        def __init__(self, ral, operator_present_topic, teleop_class):
            super().__init__(ral, operator_present_topic)
            self.set_callback(self.operator_present_cb)
            self.teleop_class = teleop_class

        def operator_present_cb(self, value):
            if value != None:
                self.teleop_class.operator_is_present = value
            self.teleop_class.operator_present(self.teleop_class.operator_is_present)

    def __init__(self, ral, master, puppet, clutch_topic, expected_interval, operator_present_topic = "", config_file_name = ""):
        print('Initialzing dvrk_teleoperation for {} and {}'.format(master.name(), puppet.name()))
        self.ral = ral
        self.expected_interval = expected_interval
        self.sleep_rate = self.ral.create_rate(int(1/expected_interval))

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

        self.clutched = False
        self.back_from_clutch = False
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
        self.operator_is_active = False
        self.tolerance_back_from_clutch = 2 * math.pi / 180
        self.master_use_measured_cv = False

        self.master_measured_cp = PyKDL.Frame()
        self.master_move_cp = PyKDL.Frame()
        self.puppet_setpoint_cp = PyKDL.Frame()
        self.puppet_servo_cp = PyKDL.Frame()
        self.puppet_jaw_servo_jp = numpy.zeros((6,))

        if operator_present_topic != "":
            self.operator_is_present = False
            self.operator_present_obj = self.operator_present_button(ral, operator_present_topic, self)
        else:
            self.operator_is_present = True # if not given, then always assume present

        self.current_state = self.state.DISABLED
        self.desired_state = self.state.DISABLED

        self.clutch_button = crtk.joystick_button(ral, clutch_topic)
        self.clutch_button.set_callback(self.clutch)

        self.startup()

    def startup(self):
        # check if functions for jaw are connected
        if not self.jaw_ignore:
            if not callable(getattr(self.puppet.jaw, "setpoint_js", None)) or not callable(getattr(self.puppet.jaw, "servo_jp", None)):
                print(f'{self.ral.node_name()}: optional functions \"jaw/servo_jp\" and \"jaw/setpoint_js\" are not connected, setting \"ignore-jaw\" to true')
                self.jaw_ignore = True

        # check if MTM has measured_cv as needed
        if self.master_use_measured_cv and not callable(getattr(self.master, "measured_cv", None)):
            self.master_use_measured_cv = False
            print(f'{self.ral.node_name()}: master ({self.master.name()} doesn\'t provide measured_cv, you can avoid this warning by setting \"use-mtm-velocity\" to false)')


    def run_all_states(self):
        self.master_measured_cp = self.master.measured_cp()
        if self.master_use_measured_cv:
            self.master_measured_cv = self.master.measured_cv()
        self.master_setpoint_cp = self.master.setpoint_cp()
        self.puppet_setpoint_cp = self.puppet.setpoint_cp()

        # TODO: add base frame (?) and check data validity

        if self.desired_state == self.state.DISABLED and self.current_state != self.state.DISABLED:
            self.set_current_state(self.state.DISABLED)

        # TODO: this doesn't seem to detect when there's a power loss on the PSM?
        if self.current_state != self.state.DISABLED and self.current_state != self.state.SETTING_ARMS_STATE:
            if not self.puppet.is_enabled() or not self.puppet.is_homed():
                self.set_desired_state(self.state.DISABLED)
                print(f'ERROR: {self.ral.node_name()}: puppet ({self.puppet.name()}) is not in state \"ENABLED\" anymore')
            if not self.master.is_enabled() or not self.master.is_homed():
                self.set_desired_state(self.state.DISABLED)
                print(f'ERROR: {self.ral.node_name()}: puppet ({self.master.name()}) is not in state \"READY\" anymore')

        self.sleep_rate.sleep()


    def transition_disabled(self):
        if self.current_state != self.desired_state:
            self.set_current_state(self.state.SETTING_ARMS_STATE)
            self.entering_state = True

    def enter_setting_arms_state(self):
        self.in_state_timer = time.perf_counter()

        if not self.puppet.is_enabled():
            self.puppet.enable()
        if not self.puppet.is_homed():
            self.puppet.home()

        if not self.master.is_enabled():
            self.master.enable()
        if not self.master.is_homed():
            self.master.home()


    def transition_setting_arms_state(self):
        if self.puppet.is_enabled() and self.puppet.is_homed() and self.master.is_enabled() and self.master.is_homed():
            self.set_current_state(self.state.ALIGNING_ARM)
            self.entering_state = True
            return
        if time.perf_counter() - self.in_state_timer > 60:
            if not (self.puppet.is_enabled() and self.puppet.is_homed()):
                print(f'ERROR: {self.ral.node_name()} timed out when setting up puppet ({self.puppet.name()}) state')
            if not (self.master.is_enabled() and self.master.is_homed()):
                print(f'ERROR: {self.ral.node_name()} timed out when setting up master ({self.master.name()}) state')
            self.set_desired_state(self.state.DISABLED)


    def enter_aligning_arm(self):
        # TODO: Update GUI to reflect scale?
        # reset timer
        self.in_state_timer = time.perf_counter()
        self.time_since_last_align = 0

        self.master.use_gravity_compensation(True)

        if self.back_from_clutch:
            self.operator_is_active = self.operator_was_active_before_clutch
            self.back_from_clutch = False

        if not self.jaw_ignore:
            self.update_gripper_to_jaw_configuration()

        self.operator_roll_min = math.pi * 100
        self.operator_roll_max = -math.pi * 100
        self.operator_gripper_min = math.pi * 100
        self.operator_gripper_max = -math.pi * 100

    def transition_aligning_arm(self):
        if self.desired_state == self.current_state:
            return

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
            roll = math.acos(PyKDL.dot(PyKDL.Vector(desired_orientation[0,1], desired_orientation[1,1], desired_orientation[2,1]),
                                       PyKDL.Vector(self.master_measured_cp.M[0,1], self.master_measured_cp.M[1,1], self.master_measured_cp.M[2,1])))
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

        # Check for actual transition
        if (orientation_error <= self.operator_orientation_tolerance) and self.operator_is_active:
            if self.desired_state == self.state.ENABLED:
                self.set_current_state(self.state.ENABLED)
                self.entering_state = True
        else:
            if time.perf_counter() - self.in_state_timer > 2:
                if orientation_error >= self.operator_orientation_tolerance:
                    print(f'{self.ral.node_name()}: unable to align master ({self.master.name()}), angle error is {orientation_error * 180 / math.pi} (deg)')
                elif not self.operator_is_active:
                    print(f'{self.ral.node_name()}: pinch/twist master ({self.master.name()}) gripper a bit')
                self.in_state_timer = time.perf_counter()


    def run_aligning_arm(self):
        # Run
        if self.clutched:
            return

        current_time = time.perf_counter()
        if current_time - self.time_since_last_align > 10:
            self.time_since_last_align = current_time
            master_cartesian_goal = PyKDL.Frame()
            master_cartesian_goal.p = self.master_setpoint_cp.p
            master_cartesian_goal.M = self.puppet_setpoint_cp.M
            self.master_move_cp = master_cartesian_goal
            handle = self.master.move_cp(self.master_move_cp)
            handle.wait()


    def enter_enabled(self):
        # update MTM/PSM previous position
        self.update_initial_state()

        # set gripper ghost if needed
        if not self.jaw_ignore:
            self.jaw_caught_up_after_clutch = False
            # gripper ghost
            self.puppet_jaw_setpoint_js = self.puppet.jaw.setpoint_js()
            if len(self.puppet_jaw_setpoint_js[0]) != 1:
                print(f'{self.ral.node_name()}: unable to get jaw position. Make sure there is an instrument on the puppet ({self.puppet.name()})')
                self.set_desired_state(self.state.DISABLED)
            current_jaw = self.puppet_jaw_setpoint_js[0][0]
            self.gripper_ghost = self.jaw_to_gripper(current_jaw)

        self.master.use_gravity_compensation(True)

        # check if by any chance the clutch pedal is pressed
        if self.clutched:
            self.clutch(True)

    def transition_enabled(self):
        if self.desired_state != self.current_state:
            self.set_current_state(self.desired_state)

    def run_enabled(self):
        if self.clutched:
            return

        if not self.master_measured_cp or not self.puppet_setpoint_cp:
            return

        # let arm move freely
        wrench = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.master.body.servo_cf(wrench)

        master_position = self.master_measured_cp

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


    def clutch(self, clutch):
        self.clutched = clutch

        if not self.current_state == self.state.ENABLED:
            return

        if clutch:
            # keep track of last follow mode
            self.operator_was_active_before_clutch = self.operator_is_active
            self.master_move_cp.M = self.puppet_setpoint_cp.M
            self.master_move_cp.p = self.master_measured_cp.p

            # let arm move freely
            wrench = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.master.body.servo_cf(wrench)

            self.master.lock_orientation(self.master.measured_cp().M)
            self.puppet.hold()
        else:
            self.set_current_state(self.state.SETTING_ARMS_STATE)
            self.back_from_clutch = True
            self.jaw_caught_up_after_clutch = False

    def operator_present(self, operator_is_present):
        if not operator_is_present:
            self.set_desired_state(self.state.ALIGNING_ARM)
        else:
            self.set_desired_state(self.state.ENABLED)

    def update_align_offset(self):
        desired_orientation = self.puppet_setpoint_cp.M
        self.alignment_offset = self.master_measured_cp.M.Inverse() * desired_orientation
        return desired_orientation

    def update_initial_state(self):
        self.master_cartesian_initial = self.master_measured_cp
        self.puppet_cartesian_initial = self.puppet_setpoint_cp
        self.update_align_offset()
        self.alignment_offset_initial = self.alignment_offset
        # TODO: missing base frame (?) here

    def set_scale(self, scale):
        self.scale = scale
        self.update_initial_state()

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

    def set_current_state(self, state):
        if state == self.state.DISABLED:
            print('Moving into state \"DISABLED\"')
        elif state == self.state.SETTING_ARMS_STATE:
            print('Moving into state \"SETTING_ARMS\"')
            self.enter_setting_arms_state()
        elif state == self.state.ALIGNING_ARM:
            print('Moving into state \"ALIGNING_ARM\"')
            self.enter_aligning_arm()
        elif state == self.state.ENABLED:
            print('Moving into state \"ENABLED\"')
            self.enter_enabled()
        else:
            raise RuntimeError('Invalid state')
        self.current_state = state

    def set_desired_state(self, state):
        self.desired_state = state
        self.operator_is_active = False

    def run(self):
        while not self.ral.is_shutdown():
            try:
                if self.current_state == self.state.DISABLED:
                    self.transition_disabled()
                elif self.current_state == self.state.SETTING_ARMS_STATE:
                    self.transition_setting_arms_state()
                elif self.current_state == self.state.ALIGNING_ARM:
                    self.transition_aligning_arm()
                elif self.current_state == self.state.ENABLED:
                    self.transition_enabled()
                self.run_all_states()
                if self.current_state == self.state.ALIGNING_ARM:
                    self.run_aligning_arm()
                elif self.current_state == self.state.ENABLED:
                    self.run_enabled()
            except Exception as e:
                print(e)
                break

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
        # self.__crtk_utils.add_measured_cv()
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
