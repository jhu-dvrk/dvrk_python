#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2024 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py -a <arm-name>

import argparse
import sys
import time
import crtk
import dvrk
import math
import numpy
import PyKDL

# example of application using arm.py
class example_application:

    # configuration
    def __init__(self, ral, arm_name, period = 0.01):
        print('configuring dvrk_arm_test for {}'.format(arm_name))
        self.ral = ral
        self.period = period
        self.arm = dvrk.arm(ral = ral,
                            arm_name = arm_name)

    # homing example
    def home(self):
        self.arm.check_connections()

        print('starting enable')
        if not self.arm.enable(10):
            sys.exit('failed to enable within 10 seconds')
        print('starting home')
        if not self.arm.home(10):
            sys.exit('failed to home within 10 seconds')
        # get current joints just to set size
        print('move to starting position')
        jp, t = self.arm.setpoint_jp()
        if t:
            goal = numpy.copy(jp)
        else:
            print('--------- not valid----')
        # go to zero position, for PSM and ECM make sure 3rd joint is past cannula
        goal.fill(0)
        if ((self.arm.name() == 'PSM1') or (self.arm.name() == 'PSM2')
            or (self.arm.name() == 'PSM3') or (self.arm.name() == 'ECM')):
            goal[2] = 0.12
        # move and wait
        print('moving to starting position')
        self.arm.move_jp(goal).wait()
        # try to move again to make sure waiting is working fine, i.e. not blocking
        print('testing move to current position')
        move_handle = self.arm.move_jp(goal)
        print('move handle should return immediately')
        move_handle.wait()
        print('home complete')

    # get methods
    def run_get(self):
        p, v, e, t = self.arm.measured_js()
        d, t = self.arm.measured_jp()
        d, t = self.arm.measured_jv()
        d, t = self.arm.measured_jf()
        d, t = self.arm.measured_cp()
        d, t = self.arm.local.measured_cp()
        d, t = self.arm.measured_cv()
        d, t = self.arm.body.measured_cf()
        d, t = self.arm.spatial.measured_cf()

        p, v, e, t = self.arm.setpoint_js()
        d, t = self.arm.setpoint_jp()
        d, t  = self.arm.setpoint_jv()
        d, t  = self.arm.setpoint_jf()
        d, t = self.arm.setpoint_cp()
        d, t = self.arm.local.setpoint_cp()

    # direct joint control example
    def run_servo_jp(self):
        print('starting servo_jp')
        # get current position
        jp, t = self.arm.setpoint_jp()
        print(jp)
        initial_joint_position = numpy.copy(jp)
        print('testing direct joint position for 2 joints out of %i' % initial_joint_position.size)
        amplitude = math.radians(5.0) # +/- 5 degrees
        duration = 5  # seconds
        samples = duration / self.period
        # create a new goal starting with current position
        goal_p = numpy.copy(initial_joint_position)
        goal_v = numpy.zeros(goal_p.size)
        start = time.time()

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        for i in range(int(samples)):
            angle = i * math.radians(360.0) / samples
            goal_p[0] = initial_joint_position[0] + amplitude * (1.0 - math.cos(angle))
            goal_p[1] = initial_joint_position[1] + amplitude *  (1.0 - math.cos(angle))
            goal_v[0] = amplitude * math.sin(angle)
            goal_v[1] = goal_v[0]
            self.arm.servo_jp(goal_p, goal_v)
            sleep_rate.sleep()

        actual_duration = time.time() - start
        print('servo_jp complete in %2.2f seconds (expected %2.2f)' % (actual_duration, duration))

    # goal joint control example
    def run_move_jp(self):
        print('starting move_jp')
        # get current position
        jp, _ = self.arm.setpoint_jp()
        initial_joint_position = numpy.copy(jp)
        print('testing goal joint position for 2 joints out of %i' % initial_joint_position.size)
        amplitude = math.radians(10.0)
        # create a new goal starting with current position
        goal = numpy.copy(initial_joint_position)
        # first motion
        goal[0] = initial_joint_position[0] + amplitude
        goal[1] = initial_joint_position[1] - amplitude
        self.arm.move_jp(goal).wait()
        # second motion
        goal[0] = initial_joint_position[0] - amplitude
        goal[1] = initial_joint_position[1] + amplitude
        self.arm.move_jp(goal).wait()
        # back to initial position
        self.arm.move_jp(initial_joint_position).wait()
        print('move_jp complete')

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        jp, _ = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        if ((self.arm.name().endswith('PSM1')) or (self.arm.name().endswith('PSM2'))
            or (self.arm.name().endswith('PSM3')) or (self.arm.name().endswith('ECM'))):
            print('preparing for cartesian motion')
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = 0.0
            self.arm.move_jp(goal).wait()

    # direct cartesian control example
    def run_servo_cp(self):
        print('starting servo_cp')
        self.prepare_cartesian()

        # create a new goal starting with current position
        initial_cartesian_position = PyKDL.Frame()
        cp, _ = self.arm.setpoint_cp()
        initial_cartesian_position.p = cp.p
        initial_cartesian_position.M = cp.M
        goal = PyKDL.Frame()
        goal.p = cp.p
        goal.M = cp.M
        # motion parameters
        amplitude = 0.02 # 4 cm total
        duration = 5  # 5 seconds
        samples = duration / self.period
        start = time.time()

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        for i in range(int(samples)):
            goal.p[0] =  initial_cartesian_position.p[0] + amplitude *  (1.0 - math.cos(i * math.radians(360.0) / samples))
            goal.p[1] =  initial_cartesian_position.p[1] + amplitude *  (1.0 - math.cos(i * math.radians(360.0) / samples))
            self.arm.servo_cp(goal)
            # check error on kinematics, compare to desired on arm.
            # to test tracking error we would compare to
            # current_position
            setpoint_cp, _ = self.arm.setpoint_cp()
            errorX = goal.p[0] - setpoint_cp.p[0]
            errorY = goal.p[1] - setpoint_cp.p[1]
            errorZ = goal.p[2] - setpoint_cp.p[2]
            error = math.sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ)
            if error > 0.002: # 2 mm
                print('Inverse kinematic error in position [%i]: %s (might be due to latency)' % (i, error))
            sleep_rate.sleep()

        actual_duration = time.time() - start
        print('servo_cp complete in %2.2f seconds (expected %2.2f)' % (actual_duration, duration))

    # direct cartesian control example
    def run_move_cp(self):
        print('starting move_cp')
        self.prepare_cartesian()

        # create a new goal starting with current position
        initial_cartesian_position = PyKDL.Frame()
        cp, _ = self.arm.setpoint_cp()
        initial_cartesian_position.p = cp.p
        initial_cartesian_position.M = cp.M
        goal = PyKDL.Frame()
        goal.p = cp.p
        goal.M = cp.M

        # motion parameters
        amplitude = 0.05 # 5 cm

        # first motion
        goal.p[0] =  initial_cartesian_position.p[0] - amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.move_cp(goal).wait()
        # second motion
        goal.p[0] =  initial_cartesian_position.p[0] + amplitude
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.move_cp(goal).wait()
        # back to initial position
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.move_cp(goal).wait()
        # first motion
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1] - amplitude
        self.arm.move_cp(goal).wait()
        # second motion
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1] + amplitude
        self.arm.move_cp(goal).wait()
        # back to initial position
        goal.p[0] =  initial_cartesian_position.p[0]
        goal.p[1] =  initial_cartesian_position.p[1]
        self.arm.move_cp(goal).wait()
        print('move_cp complete')

    # main method
    def run(self):
        self.home()
        self.run_get()
        self.run_servo_jp()
        self.run_move_jp()
        self.run_servo_cp()
        self.run_move_cp()

if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', type = str, required = True,
                        choices=['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                        help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')
    parser.add_argument('-p', '--period', type =float, default = 0.01,
                        help = 'period used for loops using servo commands')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_arm_test')
    application = example_application(ral, args.arm, args.period)
    ral.spin_and_execute(application.run)
