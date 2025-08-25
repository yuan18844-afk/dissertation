#!/usr/bin/env python
import rospy
import csv
import os
from sensor_msgs.msg import JointState

def callback(msg):
    if callback.first:
        callback.writer.writerow(['time'] + list(msg.name))
        callback.first = False
    row = [rospy.get_time()] + list(msg.position)
    callback.writer.writerow(row)

def listener(topic_name, output_file):
    rospy.init_node('record_joint_states_{}'.format(topic_name.replace('/', '')), anonymous=True)
    output_path = os.path.expanduser(output_file)
    with open(output_path, 'w') as csvfile:
        callback.writer = csv.writer(csvfile)
        callback.first = True
        rospy.Subscriber(topic_name, JointState, callback)
        rospy.loginfo("Recording joint states from {} to {}".format(topic_name, output_file))
        rospy.spin()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', type=str, default='/joint_states')
    parser.add_argument('--output', type=str, required=True)
    args = parser.parse_args()
    listener(args.topic, args.output)

