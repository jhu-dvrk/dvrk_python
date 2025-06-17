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
import sys
import time
import crtk
import dvrk

# example of application using console.py
class example_application:

    # configuration
    def __init__(self, ral, console_name):
        print('configuring dvrk_console_test')
        self.ral = ral
        self.console = dvrk.console(ral = ral, console_name = console_name)
        self.ral.check_connections()

    # main method
    def run(self):
        print('starting teleoperation')
        self.console.teleop_start()
        input('press [enter] once teleoperation is turned on')

        print('stopping teleoperation')
        self.console.teleop_stop()
        input('press [enter] once teleoperation is turned off')

        current_scale = self.console.teleop_get_scale()
        if current_scale > 0.5:
            new_scale = round(current_scale - 0.1, 2)
        else:
            new_scale = round(current_scale + 0.1, 2)
        print(f'current teleoperation scale is {current_scale}, setting it to {new_scale}')
        self.console.teleop_set_scale(new_scale)
        input('press [enter] once the scale has been changed in dVRK system GUI')
        current_scale = self.console.teleop_get_scale()
        print(f'current teleoperation scale is {current_scale}, expected {new_scale}')

if __name__ == '__main__':
    # extract ros arguments (e.g. __ns:= for namespace)
    argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name
    # parse arguments

    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--console', type = str, required = True,
                        help = 'console name as seen in the dVRK system application GUI or in ROS topics')
    args = parser.parse_args(argv)

    ral = crtk.ral('dvrk_console_test')
    application = example_application(ral, args.console)
    ral.spin_and_execute(application.run)
