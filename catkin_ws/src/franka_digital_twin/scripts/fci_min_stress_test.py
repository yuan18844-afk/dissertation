#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

current_joints = None
def joint_cb(msg):
    global current_joints
    current_joints = msg.position

if __name__ == "__main__":
    rospy.init_node("fci_min_stress_test")

    rospy.Subscriber("/joint_states", JointState, joint_cb)
    pub = rospy.Publisher("/position_joint_trajectory_controller/command", JointTrajectory, queue_size=10)

    rospy.loginfo("Waiting for current joint state...")
    while current_joints is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    start_pos = list(current_joints)
    rospy.loginfo("Current joints: %s", start_pos)

    joints = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']
    traj = JointTrajectory()
    traj.joint_names = joints
    traj.header.stamp = rospy.Time.now()

    duration = 12.0
    steps = 300
    amp = 0.01
    hold_time = 2.0
    eps = 1e-3  

    t_accum = 0.0


    p0 = JointTrajectoryPoint()
    p0.positions = start_pos
    p0.time_from_start = rospy.Duration(t_accum)
    traj.points.append(p0)


    t_accum += hold_time + eps
    p1 = JointTrajectoryPoint()
    p1.positions = start_pos
    p1.time_from_start = rospy.Duration(t_accum)
    traj.points.append(p1)


    for i in range(steps):
        t_accum += (duration-hold_time) / steps
        pos = [start_pos[j] + amp*np.sin(2*np.pi*(i/float(steps))) for j in range(7)]
        p = JointTrajectoryPoint()
        p.positions = pos
        p.time_from_start = rospy.Duration(t_accum)
        traj.points.append(p)

    rospy.sleep(1.0)
    pub.publish(traj)
    rospy.loginfo("✅ Minimal stress trajectory sent, total points: %d", len(traj.points))

