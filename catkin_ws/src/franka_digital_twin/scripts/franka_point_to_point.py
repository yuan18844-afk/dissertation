#!/usr/bin/env python
import rospy
import actionlib
import numpy as np
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import random

current_pos = None

def joint_state_cb(msg):
    global current_pos
    current_pos = np.array(msg.position[:7])

def generate_smooth_trajectory(start, target, num_points=20, total_time=6.0):
    points = []
    times = np.linspace(0.5, total_time, num_points)
    for t in times:
        s = (t - 0.5) / (total_time - 0.5)
        s = max(0.0, min(1.0, s))
        pos = start + s * (target - start)
        vel = (target - start) / (total_time - 0.5) * 0.05  # 低速平滑
        point = JointTrajectoryPoint()
        point.positions = pos.tolist()
        point.velocities = vel.tolist()
        point.time_from_start = rospy.Duration(t)
        points.append(point)
    return points

def main():
    global current_pos
    rospy.init_node("franka_action_trajectory")

    rospy.Subscriber("/joint_states", JointState, joint_state_cb)
    rospy.loginfo("Waiting for current joint state...")
    while current_pos is None and not rospy.is_shutdown():
        rospy.sleep(0.1)
    rospy.loginfo("Current position: %s", current_pos)

    # 随机目标 ±0.3 rad
    delta = np.array([random.uniform(-0.3, 0.3) for _ in range(7)])
    target_positions = current_pos + delta
    rospy.loginfo("Target: %s", target_positions)


    traj = JointTrajectory()
    traj.joint_names = [
        "panda_joint1", "panda_joint2", "panda_joint3",
        "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
    ]
    traj.points = generate_smooth_trajectory(current_pos, target_positions, num_points=30, total_time=8.0)

    client = actionlib.SimpleActionClient(
        "/position_joint_trajectory_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction
    )
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    rospy.loginfo("Sending trajectory via FollowJointTrajectory action...")
    client.send_goal(goal)
    client.wait_for_result()
    rospy.loginfo("Trajectory execution finished.")

if __name__ == "__main__":
    main()

