#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Pose2D
from relative_move.srv import SetRelativeMove
from ar_pose.srv import Track

START_X = 0.0
START_Y = 0.0

STATION_5_X = 0.60
STATION_5_Y = 1.20
STATION_5_ID = 1

STATION_1_X = 2.00
STATION_1_Y = 2.20
STATION_1_ID = 1

class NavTestCommander:
    def __init__(self):
        rospy.init_node('nav_test_commander', anonymous=True)

        self.current_x = START_X
        self.current_y = START_Y

        print(">>> 正在连接导航服务...")
        try:
            rospy.wait_for_service('/relative_move', timeout=10)
            rospy.wait_for_service('/ar_track', timeout=10)
        except rospy.ROSException:
            print(">>> 连接超时！请检查导航节点是否启动。")
            exit(1)

        self.srv_move = rospy.ServiceProxy('/relative_move', SetRelativeMove)
        self.srv_ar_track = rospy.ServiceProxy('/ar_track', Track)

        print(">>> 导航测试系统就绪。")

    def move_to(self, x, y):
        dx = x - self.current_x
        dy = y - self.current_y
        print(f"   相对移动: dx={dx:.2f}, dy={dy:.2f}")

        goal = Pose2D()
        goal.x = dx
        goal.y = dy
        goal.theta = 0.0

        try:
            response = self.srv_move(goal, "odom", True, False)
            if response.success:
                self.current_x = x
                self.current_y = y
                print(f"   到达 ({x:.2f}, {y:.2f})")
                return True
            else:
                print(f"   移动失败: {response.message}")
                return False
        except Exception as e:
            print(f"   错误: {e}")
            return False

    def ar_align(self, ar_id, goal_dist=0.25):
        print(f"   对准 AR-{ar_id} (距离={goal_dist}m)...")
        try:
            response = self.srv_ar_track(ar_id=int(ar_id), goal_dist=goal_dist)
            if response.success:
                print(f"   AR-{ar_id} 对准成功!")
                return True
            else:
                print(f"   对准失败: {response.message}")
                return False
        except Exception as e:
            print(f"   错误: {e}")
            return False

    def go_to_station(self, x, y, ar_id):
        print(f"\n=== 移动到站点 ===")
        print(f"   目标: ({x:.2f}, {y:.2f})")

        self.move_to(x, y)
        time.sleep(0.5)

        if ar_id > 0:
            self.ar_align(ar_id)

        print("   到达完成")

    def auto_mission(self):
        print("\n" + "="*50)
        print("   自动导航任务")
        print("="*50)

        print("\n[1/4] 返回起点...")
        self.go_to_station(START_X, START_Y, 0)

        print("\n[2/4] 移动到5号工位...")
        self.go_to_station(STATION_5_X, STATION_5_Y, STATION_5_ID)

        print("\n[3/4] 返回起点...")
        self.go_to_station(START_X, START_Y, 0)

        print("\n[4/4] 移动到1号工位...")
        self.go_to_station(STATION_1_X, STATION_1_Y, STATION_1_ID)

        print("\n" + "="*50)
        print("   任务完成!")
        print("="*50)

    def print_status(self):
        print(f"\n当前位置: X={self.current_x:.2f} Y={self.current_y:.2f}")
        print("预设位置:")
        print(f"  起点: ({START_X}, {START_Y})")
        print(f"  5号工位: ({STATION_5_X}, {STATION_5_Y}) ID-{STATION_5_ID}")
        print(f"  1号工位: ({STATION_1_X}, {STATION_1_Y}) ID-{STATION_1_ID}")

    def run(self):
        print("\n" + "="*50)
        print("   导航调试测试")
        print("="*50)

        while True:
            self.print_status()
            print("")
            print("  1. 自动导航任务")
            print("  2. 移动到5号工位")
            print("  3. 移动到1号工位")
            print("  4. 返回起点")
            print("  5. AR码对准")
            print("  6. 移动到自定义坐标")
            print("  0. 退出")
            print("-"*50)

            choice = input("选择: ").strip()

            if choice == '1':
                self.auto_mission()

            elif choice == '2':
                self.go_to_station(STATION_5_X, STATION_5_Y, STATION_5_ID)

            elif choice == '3':
                self.go_to_station(STATION_1_X, STATION_1_Y, STATION_1_ID)

            elif choice == '4':
                self.go_to_station(START_X, START_Y, 0)

            elif choice == '5':
                ar_id = input("  输入AR码ID: ").strip()
                try:
                    ar_id = int(ar_id)
                    self.ar_align(ar_id)
                except ValueError:
                    print("无效的AR码ID")

            elif choice == '6':
                try:
                    x = float(input("  X坐标: ").strip())
                    y = float(input("  Y坐标: ").strip())
                    self.move_to(x, y)
                except ValueError:
                    print("输入无效")

            elif choice == '0':
                print("退出...")
                break

            else:
                print("无效选项")

if __name__ == "__main__":
    cmdr = NavTestCommander()
    cmdr.run()
