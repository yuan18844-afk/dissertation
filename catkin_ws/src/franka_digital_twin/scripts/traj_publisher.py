#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def main():
    rospy.init_node('panda_trajectory_publisher')

    pub = rospy.Publisher('/franka_joint_position_controller/command', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(50)  # 50Hz
    start_time = rospy.Time.now().to_sec()


    freq = 0.2  
    amp = 0.3   

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - start_time
        joint_positions = [
            amp * np.sin(2 * np.pi * freq * t + phase)
            for phase in np.linspace(0, np.pi, 7)  
        ]

        msg = Float64MultiArray(data=joint_positions)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

