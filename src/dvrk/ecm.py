#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

import numpy

class ecm(arm):
    """Simple robot API wrapping around ROS messages"""

    # initialize the robot
    def __init__(self, ral, arm_name, connection_timeout = 5.0):
        # first call base class constructor
        super().__init__(ral, arm_name, connection_timeout)

    def insert_jp(self, depth):
        "insert the tool, by moving it to an absolute depth"
        jp, _ = self.setpoint_jp()
        goal = numpy.copy(jp)
        goal[2] = depth
        return self.move_jp(goal)
