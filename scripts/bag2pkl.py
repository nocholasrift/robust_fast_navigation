#!/usr/bin/env python

"""
Read a ROS bag and re-store the data with Pickle.

Pickle data is stored in a dict (key: topic name) of lists of (stamp, message)
tuples.

The reason this script is more advanced than one might expect is because when
rosbag reads messages it dynamically generates types which are not the same as
the system ROS message types (see https://github.com/ros/ros_comm/issues/769).
With Pickle, the stored types has to be the same as the system types (in order
to load them), thus the data of the bag messages are copied to a new system type
message object.

Note that
- all the messages in the ROS bag are read to memory before being
  re-stored; and
- I abuse the fact that ROS message types have a __slots__ attribute
  specifying the data fields (not thoroughly tested and may cause errors?).

Examples:
- Extract image_raw and camera_info from /stereo/left from multiple bag files
  and write to a single pickle file:

    find . -type f -name '*.bag' -print0 | sort -z | xargs -0 bag2pkl.py -t /stereo/left/camera_info -t /stereo/left/image_raw
"""

import argparse
import collections
import genpy
import importlib
import logging
import os
import rosbag
import rospy

try:
    import cPickle as pickle
except ModuleNotFoundError:
    import pickle


def obj_from_str(module_name, class_name, *args, **kwargs):
    # Get an instance of module_name.class_name
    mod = importlib.import_module(module_name)
    obj = getattr(mod, class_name)(*args, **kwargs)
    return obj


def rewrite(x):
    if isinstance(x, (bool, int, float, complex, str, genpy.Time, genpy.Duration, rospy.Time, rospy.Duration)):
        # A primitive type (see http://wiki.ros.org/msg)
        return x
    elif isinstance(x, list):
        return [rewrite(item) for item in x]
    elif isinstance(x, tuple):
        return tuple(rewrite(item) for item in x)
    elif hasattr(x, '_type') and hasattr(x, '__slots__'):
        # A ROS message type
        module_name, class_name = x._type.split('/')
        y = obj_from_str(module_name + '.msg', class_name)

        assert x.__slots__ == y.__slots__

        # Recursively rewrite fields
        for slot in x.__slots__:
            setattr(y, slot, rewrite(getattr(x, slot)))

        return y
    else:
        raise NotImplementedError("Type '{}' not handled".format(type(x)))


logging.basicConfig(level=logging.INFO)

parser = argparse.ArgumentParser(
    description='bag2pkl.py - save data from ROS bag file(s) to Pickle file using system ROS message definitions.')
parser.add_argument('infiles', nargs='+', help='input ROS bag file(s)')
parser.add_argument('-t', '--include-topic', action='append',
    help='Topic to include (excluding others). Provide the option multiple times to include more topics.')
parser.add_argument('-o', '--outfile', help='Output Pickle file (defaults to ${infiles[0]}.pkl)')
args = parser.parse_args()

if args.outfile is None:
    args.outfile = os.path.join(os.getcwd(), os.path.splitext(os.path.basename(args.infiles[0]))[0] + '.pkl')

msgs = collections.defaultdict(list)

for file in args.infiles:
    logging.info("Reading '{}'".format(file))

    with rosbag.Bag(file) as bag:
        # Check that requested topics are actually in the bag
        if args.include_topic:
            topics_diff = list(set(args.include_topic) - set(bag.get_type_and_topic_info()[1].keys()))

            if topics_diff:
                logging.warn("Topics {} does not exist bag file".format(topics_diff))

        # Read messages to memory
        for topic, msg, stamp in bag.read_messages(topics=args.include_topic):
            # stamp is already an instance of rospy.rostime.Time
            msgs[topic].append((stamp, rewrite(msg)))

# Make sure messages are sorted wrt. time stamp
for topic in msgs:
    msgs[topic].sort(key=lambda x: x[0])

# Write messages to Pickle file
logging.info("Writing '{}'".format(args.outfile))

with open(args.outfile, 'wb') as pkl:
    pickle.dump(msgs, pkl, pickle.HIGHEST_PROTOCOL)
