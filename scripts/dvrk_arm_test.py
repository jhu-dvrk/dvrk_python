#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_system -j <system-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun dvrk_python dvrk_arm_test.py -a <arm-name>

import argparse
import time
import sys
import crtk
import math
import numpy
import PyKDL

class device:
    def __init__(self, ral, arm_name, connection_timeout = 5.0):
        # populate this class with all the ROS topics we need
        self.__ral = ral.create_child(arm_name)
        self.crtk_utils = crtk.utils(self, self.__ral, connection_timeout)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_setpoint_js()
        self.crtk_utils.add_measured_js()
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_servo_jp()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_servo_cp()
        self.crtk_utils.add_move_cp()

    def ral(self):
        return self.__ral

# example of application using arm.py
class example_application:

    # configuration
    def __init__(self, ral, arm_name, period = 0.01):
        print('> configuring dvrk_arm_test for {}'.format(arm_name))
        self.ral = ral
        self.arm_name = arm_name
        self.period = period
        self.arm = device(ral = ral,
                          arm_name = arm_name)
        time.sleep(0.2)

    # homing example
    def home(self):
        self.ral.check_connections()

        print('> starting enable')
        if not self.arm.enable(10):
            print('  ! failed to enable within 10 seconds')
            self.ral.shutdown()
        print('> starting home')
        if not self.arm.home(10):
            print('  ! failed to home within 10 seconds')
            self.ral.shutdown()
        # get current joints just to set size, ignore timestamp
        print('> move to starting position')
        self.prepare_cartesian()
        # move and wait
        print('> moving to starting position')
        jp, _ = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        self.arm.move_jp(goal).wait()
        # try to move again to make sure waiting is working fine, i.e. not blocking
        print('> testing move to current position')
        move_handle = self.arm.move_jp(goal)
        print('  move handle should return immediately')
        move_handle.wait()
        print('< home complete')

    # direct joint control example
    def run_servo_jp(self):
        print('> starting servo_jp')
        # get current position
        jp, t = self.arm.setpoint_jp()
        initial_joint_position = numpy.copy(jp)
        print('  testing direct joint position for 2 joints out of %i' % initial_joint_position.size)
        amplitude = math.radians(5.0) # +/- 5 degrees
        duration = 5  # seconds
        samples = duration / self.period
        # create a new goal starting with current position
        goal_p = numpy.copy(initial_joint_position)
        goal_v = numpy.zeros(goal_p.size)
        start = time.time()

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        print('  servo_jp expected duration: %2.5f seconds' % (duration))
        for i in range(int(samples)):
            angle = i * math.radians(360.0) / samples
            # position
            goal_p[0] = initial_joint_position[0] + amplitude * (1.0 - math.cos(angle))
            goal_p[1] = initial_joint_position[1] + amplitude *  (1.0 - math.cos(angle))
            # velocity is easy to compute
            goal_v[0] = amplitude * math.sin(angle)
            goal_v[1] = goal_v[0]
            self.arm.servo_jp(goal_p, goal_v)
            # get some data just to make sure the server is still
            # running, this will raise a timeout exception if the dVRK
            # died.  We also check timestamp to make sure it's valid
            _, ts = self.arm.measured_jp()
            if ts == 0:
                print('  ! received invalid data, maybe the arm is not ready anymore?')
                self.ral.shutdown()
            sleep_rate.sleep()

        actual_duration = time.time() - start
        print('< servo_jp complete in %2.5f seconds' % (actual_duration))

    # goal joint control example
    def run_move_jp(self):
        print('> starting move_jp')
        # get current position
        jp, _ = self.arm.setpoint_jp()
        initial_joint_position = numpy.copy(jp)
        print('  testing goal joint position for 2 joints out of %i' % initial_joint_position.size)
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
        print('< move_jp complete')

    # utility to position tool/camera deep enough before cartesian examples
    def prepare_cartesian(self):
        # make sure the camera is past the cannula and tool vertical
        jp, ts = self.arm.setpoint_jp()
        goal = numpy.copy(jp)
        if ((self.arm_name.endswith('PSM1')) or (self.arm_name.endswith('PSM2'))
            or (self.arm_name.endswith('PSM3')) or (self.arm_name.endswith('ECM'))):
            print('  > preparing for cartesian motion')
            # set in position joint mode
            goal[0] = 0.0
            goal[1] = 0.0
            goal[2] = 0.12
            goal[3] = 0.0
            self.arm.move_jp(goal).wait()
            print('  < ready for cartesian mode')

    # direct cartesian control example
    def run_servo_cp(self):
        print('> starting servo_cp')
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
        duration = 5  # seconds
        samples = duration / self.period
        start = time.time()

        sleep_rate = self.ral.create_rate(1.0 / self.period)
        print('  servo_cp expected duration: %2.5f seconds' % (duration))
        for i in range(int(samples)):
            goal.p[0] =  initial_cartesian_position.p[0] + amplitude *  (1.0 - math.cos(i * math.radians(360.0) / samples))
            goal.p[1] =  initial_cartesian_position.p[1] + amplitude *  (1.0 - math.cos(i * math.radians(360.0) / samples))
            self.arm.servo_cp(goal)
            # check error on kinematics, compare to desired on arm.
            # to test tracking error we would compare to
            # current_position
            setpoint_cp, ts = self.arm.setpoint_cp()
            if ts == 0:
                print('  ! received invalid data, maybe the arm is not ready for cartesian space.  Is there an instrument configured?')
                self.ral.shutdown()

            errorX = goal.p[0] - setpoint_cp.p[0]
            errorY = goal.p[1] - setpoint_cp.p[1]
            errorZ = goal.p[2] - setpoint_cp.p[2]
            error = math.sqrt(errorX * errorX + errorY * errorY + errorZ * errorZ)
            if error > 0.002: # 2 mm
                print('  Inverse kinematic error in position [%i]: %s (might be due to latency)' % (i, error))
            sleep_rate.sleep()

        actual_duration = time.time() - start
        print('< servo_cp complete in %2.5f seconds' % (actual_duration))

    # direct cartesian control example
    def run_move_cp(self):
        print('> starting move_cp')
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
        print('< move_cp complete')

    # main method
    def run(self):
        self.home()
        self.run_servo_jp()
        self.run_move_jp()
        self.run_servo_cp()
        self.run_move_cp()

    def on_shutdown(self):
        print ('>> illustrating user defined shutdown callback')

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
    ral.on_shutdown(application.on_shutdown)
    ral.spin_and_execute(application.run)
