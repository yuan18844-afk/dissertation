#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_plan_execute', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("panda_arm")


group.set_max_velocity_scaling_factor(0.1)
group.set_max_acceleration_scaling_factor(0.1)


pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.5
pose_target.position.y = 0.0
pose_target.position.z = 0.4
group.set_pose_target(pose_target)


plan = group.plan()
group.go(wait=True)

group.stop()
group.clear_pose_targets()

