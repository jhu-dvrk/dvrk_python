#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2021-06-24

# (C) Copyright 2021-2025 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# First collect a bag of data using rosbag while the robot is moving:
# > ros2 bag record /PSM1/setpoint_cp /PSM1/setpoint_cv /PSM1/setpoint_js /PSM1/jaw/setpoint_js
# Hit ctrl+c to stop rosbag recording

# Then start a single arm using:
# > ros2 run dvrk_robot dvrk_system -j <system-file>

# After that, you can replay the trajectory using:
# > ros2 run dvrk_python dvrk_bag_replay.py -a PSM1 -b /path-to-bag -m servo_cp

# If you have a PSM and want to also replay the jaw motion, use -j
# > ros2 run dvrk_python dvrk_bag_replay.py -a PSM1 -b /path-to-bag -m servo_cp -j

import crtk
import sys
import time
import numpy
import PyKDL
import argparse

if crtk.ral.ros_version() == 1:
    import rosbag
else:
    import rosbag2_py
    import rosidl_runtime_py.utilities
    import rclpy.serialization

import os

def dir_path(path):
    if os.path.isdir(path):
        return path
    else:
        raise argparse.ArgumentTypeError(f"readable_dir:{path} is not a valid path")

# simplified arm class to replay motion, better performance than
# dvrk.arm since we're only subscribing to topics we need
class replay_device:

    # simplified jaw class to control the jaws, will not be used without the -j option
    class __jaw_device:
        def __init__(self, ral,
                     operating_state_instance):
            self.__crtk_utils = crtk.utils(class_instance = self,
                                           ral = ral,
                                           operating_state_instance = operating_state_instance)
            self.__crtk_utils.add_move_jp()
            self.__crtk_utils.add_servo_jp()

    def __init__(self, ral, has_jaw):
        # populate this class with all the ROS topics we need
        self.__ral = ral
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_servo_jp()
        self.crtk_utils.add_move_jp()
        self.crtk_utils.add_servo_cp()
        self.crtk_utils.add_move_cp()
        if has_jaw:
            self.jaw = self.__jaw_device(ral.create_child('/jaw'),
                                         operating_state_instance = self)
    def ral(self):
        return self.__ral

if sys.version_info.major < 3:
    input = raw_input

# extract ros arguments (e.g. __ns:= for namespace)
argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-a', '--arm', type = str, required = True,
                    choices = ['ECM', 'MTML', 'MTMR', 'PSM1', 'PSM2', 'PSM3'],
                    help = 'arm name corresponding to ROS topics without namespace.  Use __ns:= to specify the namespace')

if crtk.ral.ros_version() == 1:
    parser.add_argument('-b', '--bag', type = argparse.FileType('r'), required = True,
                        help = 'ros bag containing the trajectory to replay.  The script assumes the topic to use is /<arm>/setpoint_cp.  You can change the topic used with the -t option')
else:
    parser.add_argument('-b', '--bag', type = dir_path, required = True,
                        help = 'ros bag containing the trajectory to replay.  The script assumes the topic to use is /<arm>/setpoint_cp.  You can change the topic used with the -t option')

parser.add_argument('-m', '--mode', type = str, required = True,
                    choices = ['servo_jp', 'servo_cp'],
                    help = 'topic used to send command to arm, either joint or cartesian positions')
parser.add_argument('-t', '--topic', type = str,
                    help = 'topic used in the ros bag.  If not set, the script will use /<arm>/setpoint_cp.  Other examples: /PSM1/local/setpoint_cp.  This is useful if you recorded the trajectory on a PSM with a base_frame defined in the system.json and you are replaying the trajectory on a PSM without a base_frame (e.g. default system configuration provided with PSM in kinematic simulation mode.  This option allows to use the recorded setpoints without base-frame')
parser.add_argument('-j', '--jaw', action = 'store_true',
                    help = 'specify if the PSM jaw should also be replayed.  The topic /<arm>/jaw/setpoint_js will be used to determine the jaw trajectory')

args = parser.parse_args(argv)

ral = crtk.ral('dvrk_bag_replay')
ral.spin()

is_cp = (args.mode == 'servo_cp')
has_jaw = args.jaw

# default topic to get data from the ros bag, depends on the mode
if args.topic is None:
    if is_cp:
        topic = '/' + args.arm + '/setpoint_cp'
    else:
        topic = '/' + args.arm + '/setpoint_js'
else:
    topic = args.topic

if has_jaw:
    jaw_topic = '/' + args.arm + '/jaw/setpoint_js'

# info
print('-- This script will use the topic %s\n   to replay a trajectory on arm %s' % (topic, args.arm))

# parse bag and create list of points
setpoint_time_previous = 0.0
setpoints_out_of_order = 0
setpoints = []

if has_jaw:
    jaw_setpoint_time_previous = 0.0
    jaw_setpoints_out_of_order = 0
    jaw_setpoints = []

# if mode is cartesian, compute a bounding box so user can make sure
# scale and reference frame make sense
if is_cp:
    bbmin = numpy.zeros(3)
    bbmax = numpy.zeros(3)

if crtk.ral.ros_version() == 1:

    print('-- Parsing bag %s' % (args.bag.name))
    for bag_topic, bag_message, t in rosbag.Bag(args.bag.name).read_messages():
        if bag_topic == topic:
            # check order of timestamps, drop if out of order
            setpoint_time = ral.to_sec(bag_message.header.stamp)
            if setpoint_time <= setpoint_time_previous:
                setpoints_out_of_order += 1
            else:
                # append message
                setpoints.append(bag_message)
                setpoint_time_previous = setpoint_time

        if has_jaw:
            if bag_topic == jaw_topic:
                # check order of timestamps, drop if out of order
                jaw_setpoint_time = ral.to_sec(bag_message.header.stamp)
                if jaw_setpoint_time <= jaw_setpoint_time_previous:
                    jaw_setpoints_out_of_order += 1
                else:
                    # append message
                    jaw_setpoints.append(bag_message)
                    jaw_setpoint_time_previous = jaw_setpoint_time

# ROS2
else:

    print('-- Parsing bag %s' % (args.bag))

    storage_options = rosbag2_py.StorageOptions(
        uri = args.bag,
        storage_id = 'sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format = 'cdr',
        output_serialization_format = 'cdr')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # find messages and topics types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # parse bag
    while reader.has_next():
        (bag_topic, bag_data, t) = reader.read_next()
        if bag_topic == topic:
            # deserialize
            msg_type = rosidl_runtime_py.utilities.get_message(type_map[bag_topic])
            bag_message = rclpy.serialization.deserialize_message(bag_data, msg_type)
            # check order of timestamps, drop if out of order
            setpoint_time = ral.to_sec(bag_message.header.stamp)
            if setpoint_time <= setpoint_time_previous:
                setpoints_out_of_order += 1
            else:
                # append message
                setpoints.append(bag_message)
                setpoint_time_previous = setpoint_time

        if has_jaw:
            if bag_topic == jaw_topic:
                # deserialize
                msg_type = rosidl_runtime_py.utilities.get_message(type_map[bag_topic])
                bag_message = rclpy.serialization.deserialize_message(bag_data, msg_type)
                # check order of timestamps, drop if out of order
                jaw_setpoint_time = ral.to_sec(bag_message.header.stamp)
                if jaw_setpoint_time <= jaw_setpoint_time_previous:
                    jaw_setpoints_out_of_order += 1
                else:
                    # append message
                    jaw_setpoints.append(bag_message)
                    jaw_setpoint_time_previous = jaw_setpoint_time


print('-- Found %i setpoints using topic %s' % (len(setpoints), topic))
if len(setpoints) == 0:
    print ('-- No trajectory found!')
    sys.exit()

# report out of order setpoints
if setpoints_out_of_order > 0:
    print('-- Found and removed %i out of order setpoints' % (setpoints_out_of_order))

# same thing for jaws
if has_jaw:
    print('-- Found %i jaw setpoints using topic %s' % (len(jaw_setpoints), jaw_topic))
    if len(jaw_setpoints) == 0:
        print ('-- No jaw trajectory found!')
        sys.exit()
    if jaw_setpoints_out_of_order > 0:
        print('-- Found and removed %i out of order jaw setpoints' % (jaw_setpoints_out_of_order))

# compute bounding box
if is_cp:
    first = True
    for setpoint in setpoints:
        position = numpy.array([setpoint.pose.position.x,
                                setpoint.pose.position.y,
                                setpoint.pose.position.z])
        if first:
            bbmin = position
            bbmax = position
            first = False
        else:
            bbmin = numpy.minimum(bbmin, position)
            bbmax = numpy.maximum(bbmax, position)

    bbmin = bbmin * 1000.0
    bbmax = bbmax * 1000.0
    print ('-- Range of motion in mm:\n   X:[%f, %f]\n   Y:[%f, %f]\n   Z:[%f, %f]\n  Make sure these values make sense.  If the ros bag was based on a different system configuration, the base frame might have changed and the trajectory might not be safe nor feasible.'
           % (bbmin[0], bbmax[0], bbmin[1], bbmax[1], bbmin[2], bbmax[2]))

# compute duration
duration = ral.to_sec(setpoints[-1].header.stamp) - ral.to_sec(setpoints[0].header.stamp)
print ('-- Duration of trajectory: %f seconds' % (duration))

# send trajectory to arm
arm = replay_device(ral.create_child(args.arm), has_jaw)
arm.ral().check_connections()

# make sure the arm is powered
print('-- Enabling arm')
if not arm.enable(10):
    sys.exit('-- Failed to enable within 10 seconds')

print('-- Homing arm')
if not arm.home(10):
    sys.exit('-- Failed to home within 10 seconds')

if is_cp:
    input('-> Make sure the arm is ready to move using cartesian positions.  For a PSM or ECM, you need to have a tool in place and the tool tip needs to be outside the cannula.  You might have to manually adjust your arm.  Press "Enter" when the arm is ready.')

input('-> Press "Enter" to move to start position')

# move to the first position using arm trajectory generation (move_)
if is_cp:
    arm.move_cp(crtk.msg_conversions.FrameFromPoseMsg(setpoints[0].pose)).wait()
else:
    arm.move_jp(numpy.array(setpoints[0].position)).wait()

if has_jaw:
    arm.jaw.move_jp(numpy.array(jaw_setpoints[0].position)).wait()

# replay
input('-> Press "Enter" to replay trajectory')

last_bag_time = ral.to_sec(setpoints[0].header.stamp)

counter = 0
if has_jaw:
    total = min(len(setpoints), len(jaw_setpoints))
else:
    total = len(setpoints)

start_time = time.time()

# for the replay, use the jp/cp setpoint for the arm to control the
# execution time.  Jaw positions are picked in order without checking
# time.  There might be better ways to synchronized the two
# sequences...
for index in range(total):
    # record start time
    loop_start_time = time.time()
    # compute expected dt
    new_bag_time = ral.to_sec(setpoints[index].header.stamp)
    delta_bag_time = new_bag_time - last_bag_time
    last_bag_time = new_bag_time
    # replay
    if is_cp:
        arm.servo_cp(crtk.msg_conversions.FrameFromPoseMsg(setpoints[index].pose))
    else:
        arm.servo_jp(numpy.array(setpoints[index].position), numpy.array(setpoints[index].velocity))
    if has_jaw:
        arm.jaw.servo_jp(numpy.array(jaw_setpoints[index].position), numpy.array(jaw_setpoints[index].velocity))
    # update progress
    counter = counter + 1
    sys.stdout.write('\r-- Progress %02.1f%%' % (float(counter) / float(total) * 100.0))
    sys.stdout.flush()
    # try to keep motion synchronized
    loop_end_time = time.time()
    sleep_time = delta_bag_time - (loop_end_time - loop_start_time)
    # if process takes time larger than system rate, don't sleep
    if sleep_time > 0:
        time.sleep(sleep_time)

ral.shutdown()

print('\n--> Time to replay trajectory: %f seconds' % (time.time() - start_time))
print('--> Done!')
