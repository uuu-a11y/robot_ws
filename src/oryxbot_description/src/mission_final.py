#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from arm_controller.srv import PickPlace, move
from arm_controller.msg import control
from ar_pose.srv import Track

# ================= 导航坐标 (单位: m) =================
START_X = 0.0
START_Y = 0.0

STATION_5_X = 0.60
STATION_5_Y = 1.20
STATION_5_AR_ID = 1

STATION_1_X = 2.00
STATION_1_Y = 2.20
STATION_1_AR_ID = 1

# ================= 机械臂坐标 (单位: mm) =================
SAFE_X = 150
SAFE_Y = 0
SAFE_Z = 200

CAMERA_X = 90
CAMERA_Y = 120
CAMERA_Z = 100

TABLE_X = 80
TABLE_Y = -190
TABLE_Z = 30

# ================= AR物块ID =================
AR_TARGET_1 = 3
AR_TARGET_2 = 9

# ================= 对准距离 =================
ALIGN_DIST = 0.25

# ================= 导航等待时间 =================
NAV_WAIT_TIME = 10.0

class FinalMission:
    def __init__(self):
        rospy.init_node('final_mission', anonymous=True)

        print("=" * 60)
        print("   最终任务脚本")
        print("=" * 60)
        print()

        print(">>> 正在连接服务...")
        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=15)
            rospy.wait_for_service('/ar_track', timeout=15)
            rospy.wait_for_service('/pick_ar', timeout=15)
            rospy.wait_for_service('/goto_position', timeout=15)
            rospy.wait_for_service('/swiftpro/on', timeout=15)
            rospy.wait_for_service('/swiftpro/off', timeout=15)
        except rospy.ROSException:
            print(">>> 连接超时！请检查节点是否启动。")
            exit(1)

        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.srv_ar_track = rospy.ServiceProxy('/ar_track', Track)
        self.srv_pick_ar = rospy.ServiceProxy('/pick_ar', PickPlace)
        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)

        rospy.sleep(1)

        print(">>> 连接成功！\n")

    def navigate_to(self, x, y):
        print(f"   [SLAM导航] 目标: ({x:.2f}, {y:.2f})")

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.pub_goal.publish(goal)
        print(f"   [SLAM导航] 目标已发送，等待 {NAV_WAIT_TIME} 秒...")

        rospy.sleep(NAV_WAIT_TIME)

        print("   [SLAM导航] 等待完成")

    def ar_align(self, ar_id):
        print(f"   [AR] 对准 AR-{ar_id} (距离 {ALIGN_DIST}m)")
        try:
            response = self.srv_ar_track(ar_id=int(ar_id), goal_dist=ALIGN_DIST)
            if response.success:
                print(f"   [AR] 对准成功")
            else:
                print(f"   [AR] 对准失败: {response.message}")
        except Exception as e:
            print(f"   [AR] 错误: {e}")

        time.sleep(1)

    def move_arm(self, x, y, z):
        p = control()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.roll = 0.0
        p.pitch = 0.0
        p.yaw = 0.0

        print(f"   [机械臂] 移动到 ({x}, {y}, {z})")
        try:
            response = self.srv_arm_move(pose=p)
            if response.success:
                pass
            else:
                print(f"   [机械臂] 移动失败: {response.message}")
        except Exception as e:
            print(f"   [机械臂] 错误: {e}")

        time.sleep(1)

    def pump_on(self):
        print("   [气泵] 开启")
        try:
            self.srv_pump_on()
        except:
            pass
        time.sleep(0.5)

    def pump_off(self):
        print("   [气泵] 关闭")
        try:
            self.srv_pump_off()
        except:
            pass
        time.sleep(0.5)

    def visual_grab(self, ar_id):
        print(f"   [视觉抓取] 识别并抓取 AR-{ar_id}")

        target = control()
        target.position.x = TABLE_X
        target.position.y = TABLE_Y
        target.position.z = TABLE_Z
        target.roll = 0.0
        target.pitch = 0.0
        target.yaw = 0.0

        try:
            response = self.srv_pick_ar(number=ar_id, mode=0, pose=target)
            if response.success:
                print(f"   [视觉抓取] 抓取成功")
            else:
                print(f"   [视觉抓取] 失败: {response.message}")
        except Exception as e:
            print(f"   [视觉抓取] 错误: {e}")

        time.sleep(1)

    def run(self):
        print("任务流程:")
        print("  1. 小车SLAM导航到5号工位 -> AR对齐")
        print("  2. 机械臂抓取 AR-3 -> 放置到台面")
        print("  3. 小车SLAM导航到1号工位 -> AR对齐")
        print("  4. 机械臂抓取 AR-9 -> 放置到台面")
        print("  5. 小车SLAM导航到起点")
        print()
        print("按 Enter 开始任务...")
        raw_input = getattr(__builtins__, 'raw_input', input)
        raw_input()

        print("\n" + "=" * 60)
        print("   开始执行任务")
        print("=" * 60 + "\n")

        try:
            print("\n[阶段1] ====== 5号工位：抓取AR-3 ======")
            print()

            print(">>> 步骤1: SLAM导航到5号工位")
            self.navigate_to(STATION_5_X, STATION_5_Y)

            print(">>> 步骤2: AR码对准")
            self.ar_align(STATION_5_AR_ID)

            print(">>> 步骤3: 机械臂移动到摄像头位置")
            self.move_arm(CAMERA_X, CAMERA_Y, CAMERA_Z)

            print(">>> 步骤4: 视觉抓取AR-3")
            self.visual_grab(AR_TARGET_1)

            print(">>> 步骤5: 机械臂移动到台面放置位置")
            self.move_arm(TABLE_X, TABLE_Y, TABLE_Z)

            print(">>> 步骤6: 停止气泵")
            self.pump_off()

            print(">>> 步骤7: 机械臂归位")
            self.move_arm(SAFE_X, SAFE_Y, SAFE_Z)

            print("\n[阶段2] ====== 1号工位：抓取AR-9 ======")
            print()

            print(">>> 步骤8: SLAM导航到1号工位")
            self.navigate_to(STATION_1_X, STATION_1_Y)

            print(">>> 步骤9: AR码对准")
            self.ar_align(STATION_1_AR_ID)

            print(">>> 步骤10: 机械臂移动到摄像头位置")
            self.move_arm(CAMERA_X, CAMERA_Y, CAMERA_Z)

            print(">>> 步骤11: 视觉抓取AR-9")
            self.visual_grab(AR_TARGET_2)

            print(">>> 步骤12: 机械臂移动到台面放置位置")
            self.move_arm(TABLE_X, TABLE_Y, TABLE_Z)

            print(">>> 步骤13: 停止气泵")
            self.pump_off()

            print(">>> 步骤14: 机械臂归位")
            self.move_arm(SAFE_X, SAFE_Y, SAFE_Z)

            print("\n[阶段3] ====== 返回起点 ======")
            print()

            print(">>> 步骤15: SLAM导航到起点")
            self.navigate_to(START_X, START_Y)

            print("\n" + "=" * 60)
            print("   任务完成!")
            print("=" * 60)

        except Exception as e:
            print(f"\n任务执行出错: {e}")

if __name__ == "__main__":
    mission = FinalMission()
    mission.run()
