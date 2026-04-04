#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

def send_goal(x, y, theta=0.0):
    rospy.init_node('nav_goal_test')

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    import math
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)

    rospy.sleep(1.0)

    print(f"发送导航目标: x={x}, y={y}, theta={theta}")
    pub.publish(pose)
    print("目标已发送!")

if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    send_goal(x, y, theta)
