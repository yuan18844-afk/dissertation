#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

def main():
    rospy.init_node('joint_goal_publisher')
    pub = rospy.Publisher('/joint_goal_unity', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    msg = Float64MultiArray()
    # Franka Emika 7-DOF 目标关节角度（单位：弧度）
    msg.data = [0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5]

    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo(f"Published joint goal: {msg.data}")
        rate.sleep()

if __name__ == '__main__':
    main()

