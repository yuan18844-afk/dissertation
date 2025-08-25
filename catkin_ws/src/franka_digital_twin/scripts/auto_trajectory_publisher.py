#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

if __name__ == "__main__":
    rospy.init_node("auto_smooth_traj")

    pub = rospy.Publisher("/effort_joint_trajectory_controller/command", JointTrajectory, queue_size=10)

    rate = rospy.Rate(100)
    duration = 5.0 
    steps = int(duration * 100)

    
    joints = ['panda_joint1', 'panda_joint2', 'panda_joint3',
              'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

    traj = JointTrajectory()
    traj.joint_names = joints
    traj.header.stamp = rospy.Time.now()


    for i in range(steps):
        t = i / 100.0
        point = JointTrajectoryPoint()
        point.positions = [0.3*np.sin(2*np.pi*t/duration + j*0.4) for j in range(7)]
        point.time_from_start = rospy.Duration.from_sec(t)
        traj.points.append(point)

    rospy.sleep(1.0)  
    pub.publish(traj)
    rospy.loginfo("Smooth trajectory sent.")

