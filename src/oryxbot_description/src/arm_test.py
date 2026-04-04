#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
from std_srvs.srv import Empty
from arm_controller.srv import PickPlace, move
from arm_controller.msg import control
from sensor_msgs.msg import JointState

SAFE_X = 150.0
SAFE_Y = 0.0
SAFE_Z = 200.0

CAMERA_X = 90.0
CAMERA_Y = 120.0
CAMERA_Z = 100.0

TABLE_X = 80.0
TABLE_Y = -190.0
TABLE_Z = 30.0

TABLE_CAMERA_X = 110.0
TABLE_CAMERA_Y = -170.0
TABLE_CAMERA_Z = 100.0

L1 = 106.6
L2 = 13.2
L3 = 142.07
L4 = 158.81

class ArmTestCommander:
    def __init__(self):
        rospy.init_node('arm_test_commander', anonymous=True)

        self.joint_angles = [0.0, 0.0, 0.0]
        self.current_position = [150.0, 0.0, 100.0]

        print(">>> 正在连接机械臂服务...")
        try:
            rospy.wait_for_service('/goto_position', timeout=10)
            rospy.wait_for_service('/pick_ar', timeout=10)
            rospy.wait_for_service('/swiftpro/on', timeout=10)
            rospy.wait_for_service('/swiftpro/off', timeout=10)
        except rospy.ROSException:
            print(">>> 连接超时！请检查机械臂节点是否启动。")
            exit(1)

        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pick_ar = rospy.ServiceProxy('/pick_ar', PickPlace)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)

        rospy.Subscriber('/swiftpro/joint_states', JointState, self.joint_callback)

        print(">>> 机械臂测试系统就绪。")

    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            self.joint_angles[0] = msg.position[0]
            self.joint_angles[1] = msg.position[1]
            self.joint_angles[2] = msg.position[2]
            self.calculate_position()

    def calculate_position(self):
        theta1 = self.joint_angles[0]
        theta2 = self.joint_angles[1]
        theta3 = self.joint_angles[2]

        theta2_rad = math.radians(theta2 - 90)
        theta3_rad = math.radians(theta3 - 90)

        x = L3 * math.cos(theta2_rad) + L4 * math.cos(theta2_rad + theta3_rad)
        y = x * math.tan(math.radians(theta1 - 90))
        z = L3 * math.sin(theta2_rad) + L4 * math.sin(theta2_rad + theta3_rad) + L1 + L2

        self.current_position = [x, y, z]

    def move_arm(self, x, y, z, show=True):
        p = control()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.roll = 0.0
        p.pitch = 0.0
        p.yaw = 0.0
        if show:
            print(f"   移动 -> X:{x}, Y:{y}, Z:{z}")
        try:
            response = self.srv_arm_move(pose=p)
            if response.success:
                if show:
                    print(f"   OK")
                return True
            else:
                if show:
                    print(f"   失败: {response.message}")
                return False
        except Exception as e:
            if show:
                print(f"   错误: {e}")
            return False

    def pump_on(self, show=True):
        if show:
            print("   开启气泵")
        try:
            self.srv_pump_on()
        except:
            pass

    def pump_off(self, show=True):
        if show:
            print("   关闭气泵")
        try:
            self.srv_pump_off()
        except:
            pass

    def go_camera(self):
        self.move_arm(CAMERA_X, CAMERA_Y, CAMERA_Z)

    def go_table(self):
        self.move_arm(TABLE_X, TABLE_Y, TABLE_Z)

    def go_table_camera(self):
        self.move_arm(TABLE_CAMERA_X, TABLE_CAMERA_Y, TABLE_CAMERA_Z)

    def go_safe(self):
        print(f"   归位到 ({SAFE_X}, {SAFE_Y}, {SAFE_Z})")
        self.move_arm(SAFE_X, SAFE_Y, SAFE_Z)
        time.sleep(2)
        print("   归位完成")

    def visual_grab_one(self, ar_id, place_x, place_y, place_z):
        print(f"\n=== 抓取 AR-{ar_id} ===")

        print("   1. 移动到摄像头位置...")
        self.go_camera()
        time.sleep(1.5)

        target = control()
        target.position.x = place_x
        target.position.y = place_y
        target.position.z = place_z
        target.roll = 0.0
        target.pitch = 0.0
        target.yaw = 0.0

        print(f"   2. 视觉识别并抓取 AR-{ar_id}...")
        try:
            response = self.srv_pick_ar(number=ar_id, mode=0, pose=target)
            if response.success:
                print(f"   抓取成功!")
                return True
            else:
                print(f"   抓取失败: {response.message}")
                return False
        except Exception as e:
            print(f"   错误: {e}")
            return False

    def auto_grab_and_place(self, ar_ids):
        print("\n" + "="*50)
        print(f"   自动抓取流程: 抓取AR-{ar_ids} -> 放置到台面")
        print("="*50)

        success_count = 0
        failed_ids = []

        for i, ar_id in enumerate(ar_ids):
            print(f"\n[{i+1}/{len(ar_ids)}] 处理 AR-{ar_id}")

            if self.visual_grab_one(ar_id, TABLE_X, TABLE_Y, TABLE_Z):
                success_count += 1
            else:
                failed_ids.append(ar_id)

        print("\n" + "="*50)
        print("   放置物块到台面")
        print("="*50)

        print("   移动到台面放置位置...")
        self.go_table()
        time.sleep(1.5)

        print("   停止气泵...")
        self.pump_off()

        print("\n" + "="*50)
        print(f"   完成! 成功: {success_count}/{len(ar_ids)}")
        if failed_ids:
            print(f"   失败: {failed_ids}")
        print("="*50)

        time.sleep(1)
        print("   归位...")
        self.go_safe()

    def print_status(self):
        print(f"\n当前状态:")
        print(f"  关节角度: J1={self.joint_angles[0]:.1f} J2={self.joint_angles[1]:.1f} J3={self.joint_angles[2]:.1f}")
        print(f"  末端位置: X={self.current_position[0]:.1f} Y={self.current_position[1]:.1f} Z={self.current_position[2]:.1f}")

    def run(self):
        print("\n" + "="*50)
        print("   机械臂调试测试")
        print("="*50)

        while True:
            self.print_status()
            print("")
            print("  1. 自动抓取 -> 放置 (可选择物块)")
            print("  2. 快速抓取3个物块 (AR-3,5,9)")
            print("  3. 移动到指定坐标")
            print("  4. 移动到摄像头位置")
            print("  5. 移动到台面摄像头位置")
            print("  6. 移动到台面放置位置")
            print("  7. 开启气泵")
            print("  8. 关闭气泵")
            print("  9. 归位")
            print("  0. 退出")
            print("-"*50)

            choice = input("选择: ").strip()

            if choice == '1':
                ids_str = input("  输入AR码ID (如 3,5,9 或回车默认3,5,9): ").strip()
                if not ids_str:
                    ids = [3, 5, 9]
                else:
                    try:
                        ids = [int(x.strip()) for x in ids_str.split(',')]
                    except ValueError:
                        print("无效输入，使用默认 [3,5,9]")
                        ids = [3, 5, 9]
                self.auto_grab_and_place(ids)

            elif choice == '2':
                self.auto_grab_and_place([3, 5, 9])

            elif choice == '3':
                print("\n请输入目标坐标:")
                try:
                    x = float(input("  X [0-280] [默认150]: ").strip() or 150)
                    y = float(input("  Y [-278~278] [默认0]: ").strip() or 0)
                    z = float(input("  Z [0-130] [默认100]: ").strip() or 100)
                except ValueError:
                    print("输入无效")
                    continue
                self.move_arm(x, y, z)
                print("OK\n")

            elif choice == '4':
                self.go_camera()
                print("OK\n")

            elif choice == '5':
                self.go_table_camera()
                print("OK\n")

            elif choice == '6':
                self.go_table()
                print("OK\n")

            elif choice == '7':
                self.pump_on()
                print("OK\n")

            elif choice == '8':
                self.pump_off()
                print("OK\n")

            elif choice == '9':
                self.go_safe()
                print("OK 已归位\n")

            elif choice == '0':
                print("退出...")
                break

            else:
                print("无效选项")

if __name__ == "__main__":
    cmdr = ArmTestCommander()
    cmdr.run()
