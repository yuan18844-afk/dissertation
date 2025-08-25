#!/usr/bin/env python3
import rospy
import csv
import os
from sensor_msgs.msg import JointState
import rostopic

def select_topic():
    preferred_topics = ['/franka_state_controller/joint_states', '/joint_states']
    for t in preferred_topics:
        info = rostopic.get_topic_class(t, blocking=False)
        if info[0] is not None:
            rospy.loginfo("Using topic: {}".format(t))
            return t
    rospy.logwarn("No joint_states topic found, defaulting to /joint_states")
    return '/joint_states'

def callback(msg):
    if callback.first:
        callback.writer.writerow(['time'] + list(msg.name))
        callback.first = False
    now = "{:.9f}".format(rospy.Time.now().to_sec())
    row = [now] + ["{:.6f}".format(p) for p in msg.position]
    callback.writer.writerow(row)

def listener(output_file):
    topic_name = select_topic()
    rospy.init_node('record_real_joint_states', anonymous=True)
    output_path = os.path.expanduser(output_file)
    with open(output_path, 'w', newline='') as csvfile:
        callback.writer = csv.writer(csvfile)
        callback.first = True
        rospy.Subscriber(topic_name, JointState, callback)
        rospy.loginfo("Recording real joint states from {} to {}".format(topic_name, output_file))
        rospy.spin()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=str, required=True)
    args = parser.parse_args()
    listener(args.output)

