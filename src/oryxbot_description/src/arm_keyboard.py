#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tty
import termios
import sys
import math
from std_srvs.srv import Empty
from arm_controller.srv import move
from arm_controller.msg import control
from sensor_msgs.msg import JointState

STEP = 10

CAMERA_X = 90.0
CAMERA_Y = 120.0
CAMERA_Z = 100.0

TABLE_X = 80.0
TABLE_Y = -190.0
TABLE_Z = 30.0

BUFFER_FRONT_X = 110.0
BUFFER_FRONT_Y = 120.0
BUFFER_FRONT_Z = 40.0

BUFFER_BACK_X = 110.0
BUFFER_BACK_Y = 180.0
BUFFER_BACK_Z = 40.0

TABLE_CAMERA_X = 110.0
TABLE_CAMERA_Y = -170.0
TABLE_CAMERA_Z = 100.0

class ArmKeyboard:
    def __init__(self):
        rospy.init_node('arm_keyboard', anonymous=True)

        self.x = 150.0
        self.y = 0.0
        self.z = 100.0

        self.joint_angles = [0.0, 0.0, 0.0]

        print(">>> 正在连接机械臂服务...")
        try:
            rospy.wait_for_service('/goto_position', timeout=10)
            rospy.wait_for_service('/swiftpro/on', timeout=10)
            rospy.wait_for_service('/swiftpro/off', timeout=10)
        except rospy.ROSException:
            print(">>> 连接超时！")
            exit(1)

        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)

        rospy.Subscriber('/swiftpro/joint_states', JointState, self.joint_callback)

        print(">>> 就绪。")

    def joint_callback(self, msg):
        if len(msg.position) >= 3:
            self.joint_angles[0] = math.degrees(msg.position[0])
            self.joint_angles[1] = math.degrees(msg.position[1])
            self.joint_angles[2] = math.degrees(msg.position[2])

    def move(self, dx, dy, dz):
        self.x = max(0, min(280, self.x + dx))
        self.y = max(-278, min(278, self.y + dy))
        self.z = max(0, min(130, self.z + dz))

        p = control()
        p.position.x = self.x
        p.position.y = self.y
        p.position.z = self.z
        p.roll = 0.0
        p.pitch = 0.0
        p.yaw = 0.0

        try:
            response = self.srv_arm_move(pose=p)
            return response.success
        except Exception as e:
            print(f"\r错误: {e}")
            return False

    def pump_on(self):
        try:
            self.srv_pump_on()
        except:
            pass

    def pump_off(self):
        try:
            self.srv_pump_off()
        except:
            pass

    def go_camera(self):
        self.x = CAMERA_X
        self.y = CAMERA_Y
        self.z = CAMERA_Z
        self.move(0, 0, 0)

    def go_table(self):
        self.x = TABLE_X
        self.y = TABLE_Y
        self.z = TABLE_Z
        self.move(0, 0, 0)

    def go_buffer_front(self):
        self.x = BUFFER_FRONT_X
        self.y = BUFFER_FRONT_Y
        self.z = BUFFER_FRONT_Z
        self.move(0, 0, 0)

    def go_buffer_back(self):
        self.x = BUFFER_BACK_X
        self.y = BUFFER_BACK_Y
        self.z = BUFFER_BACK_Z
        self.move(0, 0, 0)

    def go_table_camera(self):
        self.x = TABLE_CAMERA_X
        self.y = TABLE_CAMERA_Y
        self.z = TABLE_CAMERA_Z
        self.move(0, 0, 0)

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
        print("   机械臂键盘控制")
        print("="*50)
        print("""
    W/S      : 前后移动 (X)
    A/D      : 左右移动 (Y)
    Q/E      : 上下移动 (Z)
    空格     : 气泵开
    F        : 气泵关
    R        : 复位到中心
    G        : 移动到摄像头位置 (90,120,100)
    T        : 移动到台面摄像头位置 (110,-170,100)
    U        : 移动到台面放置位置 (80,-190,30)
    B        : 移动到前储物槽 (110,120,40)
    N        : 移动到后储物槽 (110,180,40)
    X/Y/Z    : 快速归零 X/Y/Z
    
    1-9      : 快速设置高度 (10-90mm)
    
    Ctrl+C   : 退出
""")
        print("="*50)

        while not rospy.is_shutdown():
            print(f"\rX:{self.x:3.0f} Y:{self.y:4.0f} Z:{self.z:3.0f} | J1:{self.joint_angles[0]:6.1f} J2:{self.joint_angles[1]:6.1f} J3:{self.joint_angles[2]:6.1f}", end="", flush=True)

            key = self.read_key()

            if key == '\x03':
                print("\n退出")
                break
            elif key in ['w', 'W']:
                self.move(STEP, 0, 0)
            elif key in ['s', 'S']:
                self.move(-STEP, 0, 0)
            elif key in ['a', 'A']:
                self.move(0, -STEP, 0)
            elif key in ['d', 'D']:
                self.move(0, STEP, 0)
            elif key in ['q', 'Q']:
                self.move(0, 0, STEP)
            elif key in ['e', 'E']:
                self.move(0, 0, -STEP)
            elif key == ' ':
                self.pump_on()
                print("\r气泵 ON ", end="", flush=True)
            elif key in ['f', 'F']:
                self.pump_off()
                print("\r气泵 OFF", end="", flush=True)
            elif key in ['r', 'R']:
                self.x, self.y, self.z = 150, 0, 100
                self.move(0, 0, 0)
                print("\r已复位   ", end="", flush=True)
            elif key in ['g', 'G']:
                self.go_camera()
                print(f"\r摄像头位置 ({CAMERA_X},{CAMERA_Y},{CAMERA_Z})", end="", flush=True)
            elif key in ['t', 'T']:
                self.go_table_camera()
                print(f"\r台面摄像头 ({TABLE_CAMERA_X},{TABLE_CAMERA_Y},{TABLE_CAMERA_Z})", end="", flush=True)
            elif key in ['u', 'U']:
                self.go_table()
                print(f"\r台面放置 ({TABLE_X},{TABLE_Y},{TABLE_Z})", end="", flush=True)
            elif key in ['b', 'B']:
                self.go_buffer_front()
                print(f"\r前储物槽 ({BUFFER_FRONT_X},{BUFFER_FRONT_Y},{BUFFER_FRONT_Z})", end="", flush=True)
            elif key in ['n', 'N']:
                self.go_buffer_back()
                print(f"\r后储物槽 ({BUFFER_BACK_X},{BUFFER_BACK_Y},{BUFFER_BACK_Z})", end="", flush=True)
            elif key == 'x':
                self.x = 0
                self.move(0, 0, 0)
            elif key == 'y':
                self.y = 0
                self.move(0, 0, 0)
            elif key in ['1','2','3','4','5','6','7','8','9']:
                self.z = int(key) * 10
                self.move(0, 0, 0)
                print(f"\r高度: {self.z}mm  ", end="", flush=True)

        print()

if __name__ == "__main__":
    try:
        ArmKeyboard().run()
    except rospy.ROSInterruptException:
        pass
