#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tty
import termios
import sys
import math
from geometry_msgs.msg import Pose2D
from relative_move.srv import SetRelativeMove
from ar_pose.srv import Track

MOVE_STEP = 0.2

class NavKeyboard:
    def __init__(self):
        rospy.init_node('nav_keyboard', anonymous=True)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        print(">>> 正在连接导航服务...")
        try:
            rospy.wait_for_service('/relative_move', timeout=10)
            rospy.wait_for_service('/ar_track', timeout=10)
        except rospy.ROSException:
            print(">>> 连接超时！")
            exit(1)

        self.srv_move = rospy.ServiceProxy('/relative_move', SetRelativeMove)
        self.srv_ar_track = rospy.ServiceProxy('/ar_track', Track)

        print(">>> 就绪。")

    def move_relative(self, dx, dy, dtheta=0.0):
        goal = Pose2D()
        goal.x = dx
        goal.y = dy
        goal.theta = dtheta

        try:
            response = self.srv_move(goal, "odom", True, False)
            if response.success:
                self.current_x += dx
                self.current_y += dy
                return True
            else:
                print(f"\r移动失败: {response.message}")
                return False
        except Exception as e:
            print(f"\r错误: {e}")
            return False

    def ar_align(self, ar_id, goal_dist=0.25):
        print(f"\r对准 AR-{ar_id}...")
        try:
            response = self.srv_ar_track(ar_id=int(ar_id), goal_dist=goal_dist)
            if response.success:
                print(f"\rAR-{ar_id} 对准成功!")
                return True
            else:
                print(f"\r对准失败: {response.message}")
                return False
        except Exception as e:
            print(f"\r错误: {e}")
            return False

    def go_to_position(self, x, y):
        dx = x - self.current_x
        dy = y - self.current_y
        print(f"\r移动到 ({x}, {y}): dx={dx:.2f}, dy={dy:.2f}")
        return self.move_relative(dx, dy)

    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def run(self):
        print("\n" + "="*50)
        print("   导航键盘控制")
        print("="*50)
        print("""
    W/S      : 前后移动
    A/D      : 左右移动
    Q/E      : 旋转 (左/右)
    空格     : 停止

    G        : 对准 AR码 (ID=1)
    H        : 对准 AR码 (ID=1)
    J        : 对准 AR码 (ID=1)
    K        : 自定义AR对准 (输入ID)

    R        : 复位位置 (0,0)
    P        : 打印当前位置

    1-5      : 快速移动到预设位置

    Ctrl+C   : 退出
""")
        print("="*50)

        while not rospy.is_shutdown():
            print(f"\r位置: X={self.current_x:.2f} Y={self.current_y:.2f}", end="", flush=True)

            key = self.read_key()

            if key == '\x03':
                print("\n退出")
                break
            elif key in ['w', 'W']:
                self.move_relative(MOVE_STEP, 0, 0)
            elif key in ['s', 'S']:
                self.move_relative(-MOVE_STEP, 0, 0)
            elif key in ['a', 'A']:
                self.move_relative(0, -MOVE_STEP, 0)
            elif key in ['d', 'D']:
                self.move_relative(0, MOVE_STEP, 0)
            elif key in ['q', 'Q']:
                self.move_relative(0, 0, 0.5)
            elif key in ['e', 'E']:
                self.move_relative(0, 0, -0.5)
            elif key == ' ':
                print("\r停止       ", end="", flush=True)
            elif key in ['g', 'G']:
                self.ar_align(3)
            elif key in ['h', 'H']:
                self.ar_align(5)
            elif key in ['j', 'J']:
                self.ar_align(9)
            elif key in ['k', 'K']:
                try:
                    ar_id = int(input("\n  输入AR码ID: ").strip())
                    self.ar_align(ar_id)
                except ValueError:
                    print("\r无效ID")
            elif key in ['r', 'R']:
                self.current_x, self.current_y = 0.0, 0.0
                print("\r位置已复位       ", end="", flush=True)
            elif key in ['p', 'P']:
                print(f"\n当前位置: X={self.current_x:.2f} Y={self.current_y:.2f}")
            elif key in ['1','2','3','4','5']:
                positions = {
                    '1': (0.5, 0.5),
                    '2': (1.0, 0.5),
                    '3': (1.5, 1.0),
                    '4': (2.0, 1.5),
                    '5': (2.5, 2.0),
                }
                pos = positions.get(key)
                if pos:
                    self.go_to_position(pos[0], pos[1])

        print()

if __name__ == "__main__":
    try:
        NavKeyboard().run()
    except rospy.ROSInterruptException:
        pass
