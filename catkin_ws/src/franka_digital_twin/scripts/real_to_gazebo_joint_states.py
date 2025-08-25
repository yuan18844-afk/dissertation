#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def callback(msg):
    pub.publish(msg)   

rospy.init_node('real_to_gazebo_joint_states')
pub = rospy.Publisher('/gazebo/joint_states', JointState, queue_size=10)
rospy.Subscriber('/real_panda/joint_states', JointState, callback)
rospy.spin()


