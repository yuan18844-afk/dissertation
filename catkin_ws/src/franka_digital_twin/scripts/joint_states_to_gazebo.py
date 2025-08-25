#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JointStateRepublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/joint_group_position_controller/command', Float64MultiArray, queue_size=10)
        rospy.Subscriber('/joint_states', JointState, self.callback)
        self.joint_order = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

    def callback(self, msg):
        positions = []
        for joint_name in self.joint_order:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                positions.append(msg.position[idx])
            else:
                rospy.logwarn_throttle(5.0, f"{joint_name} not in /joint_states")
                return  
        self.pub.publish(Float64MultiArray(data=positions))

if __name__ == '__main__':
    rospy.init_node('joint_states_to_gazebo')
    JointStateRepublisher()
    rospy.spin()

