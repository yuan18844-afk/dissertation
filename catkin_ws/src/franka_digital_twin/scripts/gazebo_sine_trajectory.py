#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

def main():
    rospy.init_node("gazebo_sine_trajectory")

    pub = rospy.Publisher("/joint_group_position_controller/command", Float64MultiArray, queue_size=10)
    rate = rospy.Rate(50)  # 50
    t_start = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t_start
        pos = 0.3 * np.sin(2 * np.pi * 0.1 * t)  # 


        msg = Float64MultiArray()
        msg.data = [pos, 0, 0, 0, 0, 0, 0]

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    main()

