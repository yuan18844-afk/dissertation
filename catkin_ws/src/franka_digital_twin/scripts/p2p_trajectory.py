#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import time

rospy.init_node("p2p_trajectory_node")
pub = rospy.Publisher("/joint_group_position_controller/command", Float64MultiArray, queue_size=10)
rate = rospy.Rate(100)


waypoints = [
    [0.0, 0, 0, 0, 0, 0, 0],
    [0.5, 0, 0, 0, 0, 0, 0],
    [-0.5, 0, 0, 0, 0, 0, 0],
    [0.2, 0.1, -0.2, 0, 0, 0, 0]
]

pause_duration = 2.0  

for point in waypoints:
    msg = Float64MultiArray(data=point)
    t_start = time.time()
    while time.time() - t_start < pause_duration and not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

