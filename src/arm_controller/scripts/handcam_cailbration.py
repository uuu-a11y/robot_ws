#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2023 reinovo, Inc. All Rights Reserved 
#
# @Time    : 2023/12/28 下午5:42
# @Author  : hmm
# @Email   : liuyuhang0531@foxmail.com
# @File    : Manipulator_calibration.py
import cv2 as cv
import rospy
import tf
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
from tf2_msgs.msg import TFMessage
from ar_track_alvar_msgs.msg import AlvarMarkers

def check_tf_available(source_frame, target_frame, timeout=1.0):
    """
    检查是否有从source_frame到target_frame的tf变换发布
    
    参数:
        source_frame: 源坐标系
        target_frame: 目标坐标系
        timeout: 超时时间(秒)
        
    返回:
        布尔值: 如果能获取到变换则返回True，否则返回False
    """
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    try:
        # 尝试获取变换，等待timeout时间
        transform = tf_buffer.lookup_transform(
            target_frame, 
            source_frame, 
            rospy.Time(0),  # 获取最新的变换
            rospy.Duration(timeout)
        )
        return True
    except (tf2_ros.LookupException, 
            tf2_ros.ConnectivityException, 
            tf2_ros.ExtrapolationException):
        # 捕获各种可能的异常，表示获取变换失败
        return False

def get_transform(target_frame, source_frame):
    """获取从source_frame到target_frame的坐标变换"""
    listener = tf.TransformListener()
    
    try:
        # 等待变换可用（最多等待10秒）
        listener.waitForTransform(
            target_frame, 
            source_frame, 
            rospy.Time(0), 
            rospy.Duration(10.0)
        )
        
        # 获取变换
        (trans, rot) = listener.lookupTransform(
            target_frame, 
            source_frame, 
            rospy.Time(0)
        )
        
        return trans, rot
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"获取变换 {source_frame} 到 {target_frame} 失败: {str(e)}")
        return None, None


def create_tf(source_frame, target_frame, trans, rot, current_time=None):
    """创建从source_frame到target_frame的变换"""
    # 确保在节点初始化后再获取当前时间
    if current_time is None:
        # 检查节点是否已初始化
        if not rospy.core.is_initialized():
            rospy.logwarn("ROS节点未初始化，使用默认时间戳")
            current_time = rospy.Time(0)
        else:
            current_time = rospy.Time.now()
        
    tf_msg = geometry_msgs.msg.TransformStamped()
    tf_msg.header.stamp = current_time
    tf_msg.header.frame_id = source_frame
    tf_msg.child_frame_id = target_frame
    
    # 修复平移参数赋值错误
    tf_msg.transform.translation.x = trans[0]
    tf_msg.transform.translation.y = trans[1]
    tf_msg.transform.translation.z = trans[2]
    
    # 旋转参数（四元数）
    tf_msg.transform.rotation.x = rot[0]
    tf_msg.transform.rotation.y = rot[1]
    tf_msg.transform.rotation.z = rot[2]
    tf_msg.transform.rotation.w = rot[3]
    
    return tf_msg


def publish_tfs(position):
    """主函数：初始化并发布所有TF变换"""
    br = tf2_ros.StaticTransformBroadcaster()
    rate = rospy.Rate(60.0)  # 10Hz
    
    while not rospy.is_shutdown():
        print("----------------------------------------------------")
        transforms = []
        msg = rospy.wait_for_message("hand_camera/ar_pose_marker",AlvarMarkers)
        current_time = rospy.Time.now()
        for marker in msg.markers: 
            id = marker.id
            trans, rot = get_transform("hand_camera_link","ar_marker_"+str(id))
            # trans, rot = get_transform("ar_marker_"+str(id),"hand_camera_link")

            transforms.append(create_tf("robot", "cp_"+str(id), position[id], [0.000, 0.000, -0.707, 0.707], current_time))
            transforms.append(create_tf("cp_"+str(id),"cp_cam_"+str(id), [trans[0],-trans[1],trans[2]], [rot[0],rot[1],rot[2],rot[3]], current_time))       
        # 发布所有变换
        br.sendTransform(transforms)
        rate.sleep()
        i = 0
        T_m,R_m = np.array([0.,0.,0.]),np.array([0.,0.,0.,0.])
        for marker in msg.markers:
            id = marker.id
            if check_tf_available("end","cp_cam_"+str(id)):
                trans, rot = get_transform("end","cp_cam_"+str(id))
                T_m = np.add(T_m,np.array(trans))
                R_m = np.add(R_m,np.array(rot))
                i += 1
        rpy_rad = R.from_quat(R_m/i).as_euler('zxy')  # 'xyz'表示旋转顺序
        print(f"trans:{T_m/i}; rot:{rpy_rad}")
        

if __name__ == '__main__':
    try:
        rospy.init_node('tf_publisher_with_functions', anonymous=True)
        position = [[],[0.202,0.016,0],[0.202,-0.014,0],[0.172,0.016,0],[0.172,-0.014,0]]
        publish_tfs(position)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"发生未知错误: {str(e)}")