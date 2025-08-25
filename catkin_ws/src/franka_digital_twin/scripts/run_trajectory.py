#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def generate_sine_trajectory(t, amp=0.3, freq=0.2):
   
    return [
        amp * np.sin(2 * np.pi * freq * t + phase)
        for phase in np.linspace(0, np.pi, 7)
    ]

def main():
    rospy.init_node('franka_trajectory_runner')

    pub = rospy.Publisher('/franka_joint_position_controller/command', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(50)  # 50Hz 
    start_time = rospy.Time.now().to_sec()

    duration = 20.0  # s

    rospy.loginfo("开始发布轨迹指令...")

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        if t > duration:
            rospy.loginfo("轨迹运行结束。")
            break

        joint_positions = generate_sine_trajectory(t)
        msg = Float64MultiArray(data=joint_positions)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

