#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

def main():
    rospy.init_node("franka_cartesian_target", anonymous=True)

    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander("panda_arm")


    target_pose = Pose()
    target_pose.position.x = 0.4
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3


    target_pose.orientation.x = 1.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0

    arm.set_pose_target(target_pose)


    arm.set_max_velocity_scaling_factor(0.2)
    arm.set_max_acceleration_scaling_factor(0.2)


    plan = arm.plan()
    if plan:
        rospy.loginfo("Executing Cartesian plan...")
        arm.execute(plan, wait=True)
        rospy.loginfo("Done.")
    else:
        rospy.logwarn("Planning failed!")

if __name__ == "__main__":
    main()

