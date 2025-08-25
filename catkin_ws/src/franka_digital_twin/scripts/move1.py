#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math

# 初始化 MoveIt Commander 和 ROS 节点
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('panda_arc_demo', anonymous=True)

# 创建 MoveGroupCommander
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

# 降低速度和加速度（避免 FCI 触发安全模式）
group.set_max_velocity_scaling_factor(0.1)
group.set_max_acceleration_scaling_factor(0.1)

# 获取当前末端执行器位姿
start_pose = group.get_current_pose().pose


center_x = start_pose.position.x
center_y = start_pose.position.y
radius = 0.05  # 5cm
arc_angle = 60  #  60°


waypoints = []


for angle in range(0, arc_angle + 1, 5):  #  5° 一个点
    pose = geometry_msgs.msg.Pose()
    pose.position.x = center_x + radius * (1 - math.cos(math.radians(angle)))
    pose.position.y = center_y + radius * math.sin(math.radians(angle))
    pose.position.z = start_pose.position.z
    pose.orientation = start_pose.orientation
    waypoints.append(pose)


rospy.loginfo("生成的 waypoints 数量: {}".format(len(waypoints)))
(plan, fraction) = group.compute_cartesian_path(
    waypoints,
    float(0.01),
    float(0.0)
)


if fraction > 0.9:
    rospy.loginfo("圆弧轨迹规划成功，覆盖率: {:.2f}".format(fraction))
    group.execute(plan, wait=True)
else:
    rospy.logwarn("圆弧轨迹规划不完整，覆盖率: {:.2f}".format(fraction))

