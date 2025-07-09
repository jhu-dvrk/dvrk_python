#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2024-11-06

# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# To document: script to compute FK using PSM Si forward kinematics
# using physical DH, also computes GC compensation

import argparse
import crtk
import dvrk
import math
import numpy
import PyKDL
import sys
import geometry_msgs
import sensor_msgs

import cisstRobotPython

if sys.version_info.major < 3:
    input = raw_input


# example of application using arm.py
class example_application:
    def __init__(self, ral, arm_name, expected_interval, kinematic, mounting_pitch_d):
        print('configuring dvrk_psm_test for {}'.format(arm_name))

        self.ral = ral
        self.mounting_pitch_r = math.radians(mounting_pitch_d)
        self.expected_interval = expected_interval
        self.arm = dvrk.psm(ral = ral,
                            arm_name = arm_name,
                            expected_interval = expected_interval)

        self.manipulator = cisstRobotPython.robManipulator()
        print(f'Using kinematic file {kinematic}')
        self.manipulator.LoadRobot(kinematic)
        print (f'Found kinematic chain with {self.manipulator.links.size()} links')

    def home(self):
        self.ral.check_connections()

        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')

        # get current joints just to set size
        print('move to starting position')
        goal = numpy.copy(self.arm.setpoint_jp())
        # go to zero position, make sure 3rd joint is past cannula
        goal.fill(0)
        goal[2] = 0.12
        self.arm.move_jp(goal).wait()

    def compute(self):
        self.rate = self.ral.create_rate(100) # in Hz
        # position when all joints are at 0
        jp_physical = numpy.zeros(8)
        cp = numpy.zeros(shape = (4, 4)) # 4x4 matrix

        vg = numpy.array([0.0,
                          math.sin(self.mounting_pitch_r) * 9.81,
                          math.cos(self.mounting_pitch_r) * 9.81])
        print(f'gravity vector: {vg}')
        qd = numpy.zeros(8)

        cp_pub = self.ral.publisher('/test/measured_cp', geometry_msgs.msg.PoseStamped)
        measured_cp = geometry_msgs.msg.PoseStamped()
        measured_cp.header.frame_id = 'map'

        jf_pub = self.ral.publisher('/test/servo_jf', sensor_msgs.msg.JointState)
        servo_jf = sensor_msgs.msg.JointState()
        servo_jf.name = ['yaw', 'pitch', 'insertion']

        while True:
            jp_actuator = self.arm.measured_jp()
            jp_physical[0] = jp_actuator[0]
            jp_physical[1] = 0.0
            jp_physical[2] = jp_actuator[1]
            jp_physical[3] = 0.0
            jp_physical[4] = -jp_actuator[1]
            jp_physical[5] = 0.0
            jp_physical[6] = jp_actuator[1]
            jp_physical[7] = jp_actuator[2]
            # Forward kinematics
            cp = self.manipulator.ForwardKinematics(jp_physical, 8)
            measured_cp.pose.position.x = cp[0, 3]
            measured_cp.pose.position.y = cp[1, 3]
            measured_cp.pose.position.z = cp[2, 3]
            cp_pub.publish(measured_cp)
            # Gravity compensation
            efforts = self.manipulator.CCG_MDH(jp_physical, qd, vg)
            servo_jf.effort = [efforts[0], efforts[1] - efforts[2] + efforts[3], efforts[4]]
            jf_pub.publish(servo_jf)

            self.rate.sleep()

    # main method
    def run(self):
        self.home()
        self.compute()

if __name__ == '__main__':
    # strip ros arguments
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type=str, required=True,
                        choices=['PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-i', '--interval', type=float, default=0.01,
                        help = 'expected interval in seconds between messages sent by the device')
    parser.add_argument('-p', '--mounting-pitch', type=float, default=0.0,
                        help = 'mounting pitch angle in degrees')
    parser.add_argument('-k', '--kinematic',  type = argparse.FileType('r'), required = True,
                        help = 'kinematic file using json format for cisstRobot/robManipulator class')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_kinematics_to_ros')
    application = example_application(ral, args.arm, args.interval, args.kinematic.name, args.mounting_pitch)
    ral.spin_and_execute(application.run)
