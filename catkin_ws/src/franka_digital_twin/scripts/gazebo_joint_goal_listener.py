#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

pubs = []

def callback(msg):
    for i in range(min(len(msg.data), len(pubs))):
        pubs[i].publish(msg.data[i])

if __name__ == '__main__':
    rospy.init_node('gazebo_joint_goal_listener')

    for i in range(1, 8):
        topic = '/panda_joint{}_position_controller/command'.format(i)
        pubs.append(rospy.Publisher(topic, Float64MultiArray._type.split('/')[-1], queue_size=1))

    rospy.Subscriber('/joint_goal', Float64MultiArray, callback)

    rospy.loginfo("Gazebo joint_goal listener ready.")
    rospy.spin()

