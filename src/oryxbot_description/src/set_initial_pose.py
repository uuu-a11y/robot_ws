#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_initial_pose(x=0.0, y=0.0, theta=0.0):
    rospy.init_node('set_initial_pose')

    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = rospy.Time.now()

    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = 0.0

    import math
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = 0.0
    pose.pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.pose.orientation.w = math.cos(theta / 2.0)

    pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.68]

    rospy.sleep(1.0)

    print(f"设置初始位置: x={x}, y={y}, theta={theta}")
    pub.publish(pose)
    print("初始位置已设置!")

if __name__ == "__main__":
    import sys
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    theta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    set_initial_pose(x, y, theta)
