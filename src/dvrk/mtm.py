#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

import geometry_msgs.msg
from crtk_msgs.msg import CartesianImpedance

class mtm(arm):
    """Simple robot API wrapping around ROS messages
    """

    # class to contain gripper methods
    class __Gripper:
        def __init__(self, ral, connection_timeout):
            self.__crtk_utils = crtk.utils(self, ral, connection_timeout)
            self.__crtk_utils.add_measured_js()

    # initialize the robot
    def __init__(self, ral, arm_name, connection_timeout = 5.0):
        # first call base class constructor
        super().__init__(ral, arm_name, connection_timeout)
        self.gripper = self.__Gripper(self.ral().create_child('/gripper'), connection_timeout)

        # publishers
        self.__lock_orientation_publisher = self.ral().publisher('lock_orientation',
                                                                 geometry_msgs.msg.Quaternion,
                                                                 latch = True, queue_size = 1)

        self.__unlock_orientation_publisher = self.ral().publisher('unlock_orientation',
                                                                   std_msgs.msg.Empty,
                                                                   latch = True, queue_size = 1)

        self.__set_gains_publisher = self.ral().publisher('servo_ci',
                                                          CartesianImpedance,
                                                          queue_size = 10)


    def lock_orientation_as_is(self):
        "Lock orientation based on current orientation"
        current, ts = self.setpoint_cp()
        if ts:
            self.lock_orientation(current.M)
        else:
            raise RuntimeError('setpoint_cp is not valid')

    def lock_orientation(self, orientation):
        """Lock orientation, expects a PyKDL rotation matrix (PyKDL.Rotation)"""
        q = geometry_msgs.msg.Quaternion()
        q.x, q.y, q.z, q.w = orientation.GetQuaternion()
        self.__lock_orientation_publisher.publish(q)

    def unlock_orientation(self):
        "Unlock orientation"
        e = std_msgs.msg.Empty()
        self.__unlock_orientation_publisher.publish(e)

    def servo_ci(self, gains):
        self.__set_gains_publisher.publish(gains)
