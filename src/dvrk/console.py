#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import std_msgs.msg

class console(object):
    """Simple dVRK console API wrapping around ROS messages
    """

    # initialize the console
    def __init__(self, ral, console_name = 'console'):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_console(ral, console_name)

    def __init_console(self, ral, console_name):
        """Default is console name is 'console' and it would be necessary to change it only if
        you have multiple dVRK consoles"""
        # data members, event based
        self.__ral = ral.create_child(console_name)
        self.__teleop_scale = 0.0

        # publishers
        self.__teleop_enable_pub = self.__ral.publisher('/teleop/enable',
                                                        std_msgs.msg.Bool,
                                                        latch = False, queue_size = 1)
        self.__teleop_set_scale_pub = self.__ral.publisher('/teleop/set_scale',
                                                           std_msgs.msg.Float64,
                                                           latch = False, queue_size = 1)

        # subscribers
        self.__teleop_scale_sub = self.__ral.subscriber('/teleop/scale',
                                                        std_msgs.msg.Float64,
                                                        self.__teleop_scale_cb,
                                                        latch = True)

    def __teleop_scale_cb(self, data):
        """Callback for teleop scale.

        :param data: the latest scale requested for the dVRK console"""
        self.__teleop_scale = data.data

    def teleop_start(self):
        msg = std_msgs.msg.Bool()
        msg.data = True
        self.__teleop_enable_pub.publish(msg)

    def teleop_stop(self):
        msg = std_msgs.msg.Bool()
        msg.data = False
        self.__teleop_enable_pub.publish(msg)

    def teleop_set_scale(self, scale):
        msg = std_msgs.msg.Float64()
        msg.data = scale
        self.__teleop_set_scale_pub.publish(msg)

    def teleop_get_scale(self):
        return self.__teleop_scale
