#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import std_msgs.msg

class system(object):
    """Simple dVRK system API wrapping around ROS messages
    """

    # initialize the system
    def __init__(self, ral, system_namespace = 'system'):
        # base class constructor in separate method so it can be called in derived classes
        self.__init_system(ral, system_namespace)

    def __init_system(self, ral, system_name):
        """Default is system name is 'system' and it would be necessary to change it only if
        you have multiple dVRK systems"""
        # data members, event based
        self.__ral = ral.create_child(system_name)
        self.__teleop_scale = 0.0

        # publishers
        self.__power_off_pub = self.__ral.publisher('/power_off',
                                                    std_msgs.msg.Empty,
                                                    latch = False, queue_size = 1)
        self.__power_on_pub = self.__ral.publisher('/power_on',
                                                   std_msgs.msg.Empty,
                                                   latch = False, queue_size = 1)
        self.__home_pub = self.__ral.publisher('/home',
                                               std_msgs.msg.Empty,
                                               latch = False, queue_size = 1)
        self.__set_volume_pub = self.__ral.publisher('/set_volume',
                                                     std_msgs.msg.Float64,
                                                     latch = False, queue_size = 1)
        self.__beep_pub = self.__ral.publisher('/beep',
                                               std_msgs.msg.Float64MultiArray,
                                               latch = False, queue_size = 1)
        self.__string_to_speech_pub = self.__ral.publisher('/string_to_speech',
                                                           std_msgs.msg.String,
                                                           latch = False, queue_size = 1)

    def __teleop_scale_cb(self, data):
        """Callback for teleop scale.

        :param data: the latest scale requested for the dVRK system"""
        self.__teleop_scale = data.data

    def power_off(self):
        msg = std_msgs.msg.Empty()
        self.__power_off_pub.publish(msg)

    def power_on(self):
        msg = std_msgs.msg.Empty()
        self.__power_on_pub.publish(msg)

    def home(self):
        msg = std_msgs.msg.Empty()
        self.__home_pub.publish(msg)

    def set_volume(self, volume):
        msg = std_msgs.msg.Float64()
        msg.data = volume
        self.__set_volume_pub.publish(msg)

    def beep(self, duration, frequency, volume = 1.0):
        msg = std_msgs.msg.Float64MultiArray()
        msg.data = [duration, frequency, volume]
        msg.layout.data_offset = 0
        msg.layout.dim = []
        _dim = std_msgs.msg.MultiArrayDimension()
        _dim.label = 'values'
        _dim.size = 3  # Number of rows
        _dim.stride = 1
        msg.layout.dim.append(_dim)
        self.__beep_pub.publish(msg)

    def string_to_speech(self, string):
        msg = std_msgs.msg.String()
        msg.data = string
        self.__string_to_speech_pub.publish(msg)
