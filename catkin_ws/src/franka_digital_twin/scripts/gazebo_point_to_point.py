#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import time

def send_goal(pub, joint_goal):
    msg = Float64MultiArray()
    msg.data = joint_goal
    pub.publish(msg)
    rospy.loginfo(f"Sent joint goal: {joint_goal}")

if __name__ == '__main__':
    rospy.init_node('gazebo_point_to_point_controller')


    pub = rospy.Publisher('/joint_group_position_controller/command', Float64MultiArray, queue_size=10)

    rospy.sleep(1.0)  

    joint_waypoints = [
        [0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.8],
        [0.2, -0.3, 0.1, -1.5, 0.2, 1.5, 1.0],
        [0.0, -0.5, 0.0, -2.0, 0.0, 2.0, 0.8], 
    ]

    rate = rospy.Rate(0.1) 
    for goal in joint_waypoints:
        send_goal(pub, goal)
        rospy.sleep(10.0)  

    rospy.loginfo("Point-to-point movement sequence complete.")

