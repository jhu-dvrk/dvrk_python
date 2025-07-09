#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
import sys
import time
import crtk
import dvrk

# example of application using system.py
class example_application:

    # configuration
    def __init__(self, ral):
        print('configuring dvrk_system_test')
        self.ral = ral
        self.system = dvrk.system(ral = ral)
        self.ral.check_connections(timeout_seconds = 2.0)

    # main method
    def run(self):
        # print('powering off')
        # self.system.power_off()
        # input('press [enter] once the dVRK system is powered off')

        # print('powering')
        # self.system.power_on()
        # input('press [enter] once the dVRK system is powered on')

        # print('homing')
        # self.system.home()
        # input('press [enter] once all arms are homed')

        print('audio tests')
        self.system.string_to_speech('Hello D V R K');
        time.sleep(1.0)
        self.system.beep(0.2, 50000.0, 0.5) # duration, frequency, volume %
        time.sleep(0.2)

if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

    ral = crtk.ral('dvrk_system_test')
    application = example_application(ral)
    ral.spin_and_execute(application.run)
