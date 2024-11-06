#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2024-11-06

# (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>
# Run test script:
# > rosrun dvrk_python dvrk_kinematics_to_ros.py -a PSM1 -k psm-si-physical.json

import argparse
import crtk
import dvrk
import math
import numpy
import PyKDL
import sys

import cisstRobotPython

if sys.version_info.major < 3:
    input = raw_input


# example of application using arm.py
class example_application:
    def __init__(self, ral, arm_name, expected_interval, kinematic):
        print('configuring dvrk_psm_test for {}'.format(arm_name))

        self.ral = ral
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
        jp = numpy.zeros(6)
        cp = numpy.zeros(shape = (4, 4)) # 4x4 matrix
        cp = self.manipulator.ForwardKinematics(jp, 6)
        print('joint position:')
        print(jp)
        print('cartesian position:')
        print(cp)

        while True:
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
    parser.add_argument('-k', '--kinematic',  type = argparse.FileType('r'), required = True,
                        help = 'kinematic file using json format for cisstRobot/robManipulator class')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_kinematics_to_ros')
    application = example_application(ral, args.arm, args.interval, args.kinematic.name)
    ral.spin_and_execute(application.run)
