#!/usr/bin/env python3
import rospy, sys, random, yaml, os
import moveit_commander
from geometry_msgs.msg import Pose


X_MIN, X_MAX = 0.35, 0.40
Y_MIN, Y_MAX = -0.10, 0.10
Z_MIN, Z_MAX = 0.35, 0.40


READY_JOINTS = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

def random_pose():
    p = Pose()
    p.position.x = random.uniform(X_MIN, X_MAX)
    p.position.y = random.uniform(Y_MIN, Y_MAX)
    p.position.z = random.uniform(Z_MIN, Z_MAX)
    p.orientation.x = 1.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    return p

if __name__ == "__main__":
    rospy.init_node("generate_random_traj")
    moveit_commander.roscpp_initialize(sys.argv)
    arm = moveit_commander.MoveGroupCommander("panda_arm")


    arm.set_max_velocity_scaling_factor(0.05)
    arm.set_max_acceleration_scaling_factor(0.05)
    arm.set_goal_joint_tolerance(0.02)
    arm.set_goal_position_tolerance(0.005)
    arm.set_goal_orientation_tolerance(0.01)


    rospy.loginfo("Moving to ready pose...")
    arm.set_joint_value_target(READY_JOINTS)
    plan_ready = arm.plan()
    arm.execute(plan_ready[1], wait=True)
    rospy.sleep(1.0)


    waypoints = [random_pose() for _ in range(3)]
    rospy.loginfo(f"Planning Cartesian path for {len(waypoints)} random poses...")
    plan, fraction = arm.compute_cartesian_path(waypoints, 0.05, True)

    if fraction < 0.9:
        rospy.logwarn(f"Low Cartesian path fraction: {fraction}")

    traj = plan.joint_trajectory


    yaml_path = os.path.expanduser('~/catkin_ws/src/franka_digital_twin/traj/panda_traj.yaml')
    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)

    data = {
        'joint_names': traj.joint_names,
        'points': []
    }
    for p in traj.points:
        data['points'].append({
            'positions': list(p.positions),
            'velocities': list(p.velocities) if p.velocities else [0.0]*7,
            'time_from_start': {
                'secs': p.time_from_start.secs,
                'nsecs': p.time_from_start.nsecs
            }
        })

    with open(yaml_path, 'w') as f:
        yaml.dump(data, f)

    rospy.loginfo(" Trajectory saved to {yaml_path}")
    rospy.loginfo("You can now execute this trajectory on Gazebo or the real robot.")

