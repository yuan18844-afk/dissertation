#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import RobotTrajectory
from moveit_commander.planning_interface import MoveGroupInterface, PlanningSceneInterface
import sys

def initialize_moveit_commander():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("franka_moveit_python_script", anonymous=True)

    scene = PlanningSceneInterface()
    group = MoveGroupInterface("panda_arm", wait_for_servers=10.0)

    return group, scene

def set_speed_and_acceleration(group):
    group.set_max_velocity_scaling_factor(0.1)
    group.set_max_acceleration_scaling_factor(0.1)

def execute_trajectory(group):
    start_state = group.get_current_state()

    target_position = [0.0, -0.4, 0.0, -1.5, 0.0, 1.2, 0.4]
    group.set_joint_value_target(target_position)

    success = group.go(wait=True)
    
    rospy.sleep(1)
    group.stop()

    return success

def main():
    group, scene = initialize_moveit_commander()
    

    set_speed_and_acceleration(group)


    success = execute_trajectory(group)

    if success:
        rospy.loginfo("Trajectory executed successfully")
    else:
        rospy.logerr("Trajectory execution failed")

if __name__ == "__main__":
    main()

