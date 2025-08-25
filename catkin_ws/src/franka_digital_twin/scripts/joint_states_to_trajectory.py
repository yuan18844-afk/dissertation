#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3",
    "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"
]

pub = None

def joint_state_callback(msg):
    if not all(name in msg.name for name in JOINT_NAMES):
        return

    joint_positions = []
    for name in JOINT_NAMES:
        idx = msg.name.index(name)
        joint_positions.append(msg.position[idx])

    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = joint_positions
    point.time_from_start = rospy.Duration(0.1)

    traj.points.append(point)
    pub.publish(traj)

    rospy.loginfo_throttle(1.0, "[Sync] Published to Gazebo: %s" % joint_positions)

def main():
    global pub
    rospy.init_node('real_to_gazebo_joint_sync')

    pub = rospy.Publisher(
        '/gazebo/joint_group_position_controller/command',
        JointTrajectory,
        queue_size=1
    )

    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.loginfo("Joint state mirror node started. Subscribing /joint_states.")
    rospy.spin()

if __name__ == "__main__":
    main()

