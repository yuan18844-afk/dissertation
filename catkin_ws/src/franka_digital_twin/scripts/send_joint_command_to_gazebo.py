#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

class GazeboJointCommandBridge:
    def __init__(self):

        rospy.Subscriber('/joint_goal', Float64MultiArray, self.joint_callback)


        self.joint_publishers = [
            rospy.Publisher('/panda_joint1_position_controller/command', Float64, queue_size=1),
            rospy.Publisher('/panda_joint2_position_controller/command', Float64, queue_size=1),
            rospy.Publisher('/panda_joint3_position_controller/command', Float64, queue_size=1),
            rospy.Publisher('/panda_joint4_position_controller/command', Float64, queue_size=1),
            rospy.Publisher('/panda_joint5_position_controller/command', Float64, queue_size=1),
            rospy.Publisher('/panda_joint6_position_controller/command', Float64, queue_size=1),
            rospy.Publisher('/panda_joint7_position_controller/command', Float64, queue_size=1),
        ]

    def joint_callback(self, msg):
        rospy.loginfo("接收到 joint_goal: {}".format(msg.data))
        for i in range(min(7, len(msg.data))):
            self.joint_publishers[i].publish(msg.data[i])

if __name__ == '__main__':
    rospy.init_node('gazebo_joint_goal_bridge')
    bridge = GazeboJointCommandBridge()
    rospy.loginfo("Gazebo Joint Goal Bridge opening，waiting /joint_goal...")
    rospy.spin()

