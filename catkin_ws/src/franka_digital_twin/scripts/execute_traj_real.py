#!/usr/bin/env python3
import rospy, yaml
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import actionlib


TIME_SCALE = 10.0  

if __name__ == "__main__":
    rospy.init_node("execute_traj_real")

    yaml_path = "/home/yuan8868/catkin_ws/src/franka_digital_twin/traj/panda_traj.yaml"
    rospy.loginfo(f"Loading trajectory from: {yaml_path}")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    traj = JointTrajectory()
    traj.joint_names = data['joint_names']
    traj.points = []

    for p in data['points']:
        pt = JointTrajectoryPoint()
        pt.positions = p['positions']
        pt.velocities = p.get('velocities', [0.0]*7)
        secs = p['time_from_start']['secs']
        nsecs = p['time_from_start'].get('nsecs', 0)
        t = rospy.Duration(secs, nsecs)
        pt.time_from_start = rospy.Duration.from_sec(t.to_sec() * TIME_SCALE)
        traj.points.append(pt)

    rospy.loginfo(f"Trajectory has {len(traj.points)} points.")
    rospy.loginfo(f"First point: {traj.points[0].positions}")
    rospy.loginfo(f"Last point:  {traj.points[-1].positions}")

    client = actionlib.SimpleActionClient(
        '/position_joint_trajectory_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for Action Server...")
    client.wait_for_server()
    rospy.loginfo("Action Server connected.")

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = traj

    rospy.loginfo("Sending trajectory to real robot...")
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo(f"✅ Execution finished. Result: {result}")

