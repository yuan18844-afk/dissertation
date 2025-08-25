#!/usr/bin/env python
import rospy
from urdf_parser_py.urdf import URDF

def parse_limits():
    rospy.init_node('read_panda_limits', anonymous=True)

    urdf_str = rospy.get_param('/robot_description', None)
    if urdf_str is None:
        rospy.logerr("未找到 /robot_description，请先加载 Panda URDF")
        return

    robot = URDF.from_xml_string(urdf_str)

    rospy.loginfo("=== Franka Panda Joint Limits ===")
    for joint in robot.joints:
        if joint.type in ['revolute', 'prismatic']:
            name = joint.name
            lim = joint.limit
            print("关节: {}".format(name))
            print("  角度范围: [{:.4f}, {:.4f}] rad".format(lim.lower, lim.upper))
            print("  最大速度: {:.4f} rad/s".format(lim.velocity))
            print("  力矩限制: {:.2f} Nm".format(lim.effort))
            if hasattr(joint, 'safety') and joint.safety is not None:
                print("  软限制: [{:.4f}, {:.4f}] rad".format(
                    joint.safety.soft_lower_limit, joint.safety.soft_upper_limit))
            else:
                print("  软限制: 未定义")
            print("  加速度限制(固定): 15.0 rad/s²")
            print("")

if __name__ == '__main__':
    try:
        parse_limits()
    except rospy.ROSInterruptException:
        pass

