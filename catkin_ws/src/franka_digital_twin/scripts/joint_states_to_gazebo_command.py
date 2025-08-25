#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time

JOINT_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7"
]

last_pub_time = 0.0
pub_interval = 1.0 / 50.0  # 限制为 50Hz

pub = None

def joint_state_callback(msg):
    global last_pub_time

    now = rospy.get_time()
    if now - last_pub_time < pub_interval:
        return

    joint_pos = []
    for name in JOINT_NAMES:
        if name in msg.name:
            idx = msg.name.index(name)
            joint_pos.append(msg.position[idx])
        else:
            rospy.logwarn("Joint %s missing from joint_states", name)
            return

    cmd = Float64MultiArray(data=joint_pos)
    pub.publish(cmd)
    last_pub_time = now
    rospy.loginfo_throttle(1.0, "Published to Gazebo: %s", joint_pos)

def main():
    global pub
    rospy.init_node("real_to_gazebo_joint_limited_sync")
    pub = rospy.Publisher("/joint_group_position_controller/command", Float64MultiArray, queue_size=1)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()

if __name__ == "__main__":
    main()

