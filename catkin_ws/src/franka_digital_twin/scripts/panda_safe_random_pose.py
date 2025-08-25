#!/usr/bin/env python3
import sys, rospy, random
import moveit_commander
from geometry_msgs.msg import Pose

#  Panda 安全末端范围 (米)
X_MIN, X_MAX = 0.3, 0.5
Y_MIN, Y_MAX = -0.1, 0.1
Z_MIN, Z_MAX = 0.3, 0.4

# eady 姿态
READY_JOINTS = [0, -0.785, 0, -2.0, 0, 1.571, 0.785]

def random_safe_pose():

    p = Pose()
    p.position.x = random.uniform(X_MIN, X_MAX)
    p.position.y = random.uniform(Y_MIN, Y_MAX)
    p.position.z = random.uniform(Z_MIN, Z_MAX)

    p.orientation.x = 1.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    return p

def main():
    rospy.init_node("panda_ready_and_random_pose")
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander("panda_arm")

    # 
    arm.set_max_velocity_scaling_factor(0.05)
    arm.set_max_acceleration_scaling_factor(0.05)
    arm.set_goal_joint_tolerance(0.02)
    arm.set_goal_position_tolerance(0.005)
    arm.set_goal_orientation_tolerance(0.01)

    
    rospy.loginfo("Moving to ready pose...")
    arm.set_joint_value_target(READY_JOINTS)
    plan_tuple = arm.plan()
    plan = plan_tuple[1]  # 取 JointTrajectory
    arm.execute(plan, wait=True)
    rospy.sleep(1.0)

    
    for i in range(5):
        success = False
        attempts = 0
        while not success and attempts < 10:
            attempts += 1
            target_pose = random_safe_pose()
            arm.set_pose_target(target_pose)

            plan_tuple = arm.plan()
            plan = plan_tuple[1]
            if plan and len(plan.joint_trajectory.points) > 0:
                rospy.loginfo(f"[Target {i+1}] Valid pose after {attempts} tries.")
                arm.execute(plan, wait=True)
                rospy.sleep(1.0)
                success = True
            else:
                rospy.logwarn(f"[Target {i+1}] Pose not reachable, retrying...")

        if not success:
            rospy.logerr(f"[Target {i+1}] Failed after 10 attempts.")

    rospy.loginfo("All targets executed.")
    arm.stop()

if __name__ == "__main__":
    main()

