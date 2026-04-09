#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人综合控制台 - 导航 + 机械臂
Tab 切换模式, 快捷键导航到已知点, 实时显示位置
"""

import rospy
import tty
import termios
import sys
import math
import select
from geometry_msgs.msg import Pose2D, PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from arm_controller.srv import move as MoveSrv
from arm_controller.msg import control
from relative_move.srv import SetRelativeMove
from ar_pose.srv import Track
from actionlib_msgs.msg import GoalID, GoalStatusArray

# ─────────── 配置 ───────────

NAVI_POINTS = {
    "start":            {"x": 0.0, "y": 0.0},
    "station_1":        {"x": 2.0, "y": 2.2},
    "station_2":        {"x": 2.2, "y": 1.2},
    "station_3":        {"x": 2.2, "y": 0.2},
    "station_4":        {"x": 0.6, "y": 2.2},
    "station_5":        {"x": 0.6, "y": 1.2},
    "charging_station": {"x": 0.4, "y": 2.0},
}

ARM_POSITIONS = {
    "safe":        {"x": 150, "y": 0,    "z": 100},
    "camera":      {"x": 90,  "y": 120,  "z": 100},
    "table_camera":{"x": 110, "y": -170, "z": 100},
    "table_place": {"x": 110, "y": -170, "z": 15},
    "buffer_front":{"x": 110, "y": 120,  "z": 40},
    "buffer_back": {"x": 110, "y": 180,  "z": 40},
}

MOVE_STEP = 0.2   # 小车移动步长(m)
ARM_STEP  = 10    # 机械臂步长(mm)

# ─────────── 工具 ───────────

def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # 处理方向键 (ESC [ A/B/C/D)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return f'\x1b[{ch3}'
            return ch + ch2
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def clear_line():
    print('\r' + ' ' * 90 + '\r', end='', flush=True)

# ─────────── 主类 ───────────

class RobotConsole:
    def __init__(self):
        rospy.init_node('robot_console', anonymous=True)

        # 小车状态
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.nav_goal_active = False
        self.nav_reached = False
        self.nav_aborted = False

        # 机械臂状态
        self.arm_x = 150.0
        self.arm_y = 0.0
        self.arm_z = 100.0

        # 模式: 'nav' 或 'arm'
        self.mode = 'nav'

        # ROS 接口
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self._odom_cb)
        self.sub_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self._status_cb)
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)
        self.pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=5)

        print(">>> 等待服务...")
        try:
            rospy.wait_for_service('/relative_move', timeout=10)
            rospy.wait_for_service('/ar_track', timeout=10)
            rospy.wait_for_service('/goto_position', timeout=10)
            rospy.wait_for_service('/swiftpro/on', timeout=10)
            rospy.wait_for_service('/swiftpro/off', timeout=10)
        except rospy.ROSException:
            print(">>> 部分服务不可用，功能可能受限")

        self.srv_move = rospy.ServiceProxy('/relative_move', SetRelativeMove)
        self.srv_ar = rospy.ServiceProxy('/ar_track', Track)
        self.srv_arm = rospy.ServiceProxy('/goto_position', MoveSrv)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)

        rospy.sleep(0.5)
        print(">>> 就绪")

    # ── 回调 ──

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def _status_cb(self, msg):
        if not msg.status_list:
            return
        last = msg.status_list[-1]
        if last.status == 3:
            self.nav_reached = True
        elif last.status in [4, 5, 8]:
            self.nav_aborted = True

    # ── 小车操作 ──

    def move_relative(self, dx, dy, dtheta=0.0):
        goal = Pose2D(dx, dy, dtheta)
        try:
            resp = self.srv_move(goal, "odom", True, False)
            return resp.success
        except Exception as e:
            print(f"\r  移动失败: {e}")
            return False

    def send_vel(self, vx, vy, vw):
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = vw
        self.pub_cmd.publish(t)

    def navigate_to(self, name):
        if name not in NAVI_POINTS:
            print(f"\r  未知导航点: {name}")
            return
        pt = NAVI_POINTS[name]
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = pt['x']
        goal.pose.position.y = pt['y']
        goal.pose.position.z = 0
        # 保持当前朝向
        half = self.robot_yaw / 2
        goal.pose.orientation.z = math.sin(half)
        goal.pose.orientation.w = math.cos(half)

        self.nav_reached = False
        self.nav_aborted = False
        self.pub_goal.publish(goal)
        print(f"\r  >> 导航到 {name} ({pt['x']}, {pt['y']}) ...")

    def cancel_nav(self):
        self.pub_cancel.publish(GoalID())
        print("\r  取消导航")

    def ar_align(self, ar_id, dist=0.25):
        print(f"\r  对准 AR-{ar_id} ...")
        try:
            resp = self.srv_ar(ar_id=int(ar_id), goal_dist=dist)
            if resp.success:
                print(f"\r  AR-{ar_id} 对准成功")
            else:
                print(f"\r  对准失败: {resp.message}")
        except Exception as e:
            print(f"\r  错误: {e}")

    # ── 机械臂操作 ──

    def arm_move(self, dx, dy, dz):
        self.arm_x = max(0, min(280, self.arm_x + dx))
        self.arm_y = max(-278, min(278, self.arm_y + dy))
        self.arm_z = max(0, min(130, self.arm_z + dz))
        self._arm_goto()

    def arm_goto(self, name):
        if name not in ARM_POSITIONS:
            print(f"\r  未知位置: {name}")
            return
        p = ARM_POSITIONS[name]
        self.arm_x, self.arm_y, self.arm_z = p['x'], p['y'], p['z']
        self._arm_goto()
        print(f"\r  >> 移到 {name} ({p['x']}, {p['y']}, {p['z']})")

    def _arm_goto(self):
        p = control()
        p.position.x = self.arm_x
        p.position.y = self.arm_y
        p.position.z = self.arm_z
        p.roll = p.pitch = p.yaw = 0.0
        try:
            self.srv_arm(pose=p)
        except Exception as e:
            print(f"\r  机械臂错误: {e}")

    # ── 显示 ──

    def show_status(self):
        yaw_deg = math.degrees(self.robot_yaw)
        if self.mode == 'nav':
            dist_home = math.sqrt(self.robot_x**2 + self.robot_y**2)
            tag = "导航"
            print(f"\r  [{tag}] 位置:({self.robot_x:+.2f},{self.robot_y:+.2f}) 朝向:{yaw_deg:+.0f}° 距原点:{dist_home:.2f}m  ", end='', flush=True)
        else:
            tag = "机械臂"
            print(f"\r  [{tag}] X:{self.arm_x:3.0f} Y:{self.arm_y:4.0f} Z:{self.arm_z:3.0f} (mm)  ", end='', flush=True)

    def show_help(self):
        print("""
┌─────────────────────────────────────────────────────┐
│            机器人综合控制台 v1.0                      │
├─────────────────────────────────────────────────────┤
│  Tab        切换 导航/机械臂 模式                     │
│  Ctrl+C     退出                                     │
├─────────────── 导航模式 (NAV) ──────────────────────│
│  W/S/A/D    前/后/左/右 移动 (0.2m)                   │
│  Q/E        左/右 旋转 (0.5rad)                      │
│  Shift+方向  连续发送速度指令(按住)                    │
│  空格        急停                                     │
│  1~6        导航到: 1=start 2~6=s1~s5                │
│  0          导航到充电站                              │
│  C          取消当前导航                              │
│  G+数字     AR对准 (例: G3 = 对准AR-3)                │
│  R          重置里程计位置显示                        │
├─────────────── 机械臂模式 (ARM) ───────────────────│
│  W/S        X轴 增/减 (前后)                          │
│  A/D        Y轴 增/减 (左右)                          │
│  Q/E        Z轴 增/减 (上下)                          │
│  空格        气泵开  F=气泵关                          │
│  1~6        快速移到预设位:                            │
│              1=safe 2=camera 3=table_camera           │
│              4=table_place 5=buffer_front 6=buffer_back│
│  R          复位到 safe (150,0,100)                   │
│  +/-         步长调节 (5/10/20mm)                     │
├─────────────── 通用 ──────────────────────────────│
│  P          打印当前位置详情                          │
│  H/?        显示此帮助                               │
└─────────────────────────────────────────────────────┘""")

    # ── 主循环 ──

    def run(self):
        self.show_help()
        global ARM_STEP

        nav_presets = {
            '1': 'start', '2': 'station_1', '3': 'station_2',
            '4': 'station_3', '5': 'station_4', '6': 'station_5',
            '0': 'charging_station',
        }
        arm_presets = {
            '1': 'safe', '2': 'camera', '3': 'table_camera',
            '4': 'table_place', '5': 'buffer_front', '6': 'buffer_back',
        }

        while not rospy.is_shutdown():
            self.show_status()
            key = read_key()

            # ── 通用按键 ──
            if key == '\x03':  # Ctrl+C
                print("\n退出")
                break
            elif key == '\t':  # Tab 切换模式
                self.mode = 'arm' if self.mode == 'nav' else 'nav'
                tag = "导航模式" if self.mode == 'nav' else "机械臂模式"
                print(f"\r  >> 切换到 {tag}                    ")
                continue
            elif key in ['h', 'H', '?']:
                self.show_help()
                continue
            elif key in ['p', 'P']:
                yaw_deg = math.degrees(self.robot_yaw)
                print(f"\n  ── 小车 ──")
                print(f"  位置: ({self.robot_x:.3f}, {self.robot_y:.3f})  朝向: {yaw_deg:.1f}°")
                print(f"  ── 机械臂 ──")
                print(f"  末端: X={self.arm_x:.0f} Y={self.arm_y:.0f} Z={self.arm_z:.0f} (mm)")
                print(f"  步长: {ARM_STEP}mm")
                continue

            # ── 导航模式 ──
            if self.mode == 'nav':
                if key in ['w', 'W']:
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
                    self.send_vel(0, 0, 0)
                    self.cancel_nav()
                    print("\r  急停 + 取消导航               ")
                elif key == 'c':
                    self.cancel_nav()
                elif key == 'r':
                    print(f"\r  当前位置: ({self.robot_x:.3f}, {self.robot_y:.3f}, {math.degrees(self.robot_yaw):.1f}°)")
                elif key == 'g':
                    # AR 对准
                    print("\r  输入AR码ID: ", end='', flush=True)
                    try:
                        old = termios.tcgetattr(sys.stdin.fileno())
                        tty.setcbreak(sys.stdin.fileno())
                        ar_id_str = ''
                        while True:
                            ch = sys.stdin.read(1)
                            if ch == '\n' or ch == '\r':
                                break
                            elif ch == '\x03':
                                ar_id_str = ''
                                break
                            elif ch.isdigit():
                                ar_id_str += ch
                                print(ch, end='', flush=True)
                        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
                        if ar_id_str:
                            self.ar_align(int(ar_id_str))
                    except:
                        print("\r  输入错误")
                elif key in nav_presets:
                    self.navigate_to(nav_presets[key])
                # 方向键连续速度
                elif key == '\x1b[A':  # Up
                    self.send_vel(0.2, 0, 0)
                elif key == '\x1b[B':  # Down
                    self.send_vel(-0.2, 0, 0)
                elif key == '\x1b[C':  # Right
                    self.send_vel(0, -0.2, 0)
                elif key == '\x1b[D':  # Left
                    self.send_vel(0, 0.2, 0)

            # ── 机械臂模式 ──
            else:
                if key in ['w', 'W']:
                    self.arm_move(ARM_STEP, 0, 0)
                elif key in ['s', 'S']:
                    self.arm_move(-ARM_STEP, 0, 0)
                elif key in ['a', 'A']:
                    self.arm_move(0, -ARM_STEP, 0)
                elif key in ['d', 'D']:
                    self.arm_move(0, ARM_STEP, 0)
                elif key in ['q', 'Q']:
                    self.arm_move(0, 0, ARM_STEP)
                elif key in ['e', 'E']:
                    self.arm_move(0, 0, -ARM_STEP)
                elif key == ' ':
                    self.srv_pump_on()
                    print("\r  气泵 ON                     ")
                elif key in ['f', 'F']:
                    self.srv_pump_off()
                    print("\r  气泵 OFF                    ")
                elif key in ['r', 'R']:
                    self.arm_goto('safe')
                elif key in arm_presets:
                    self.arm_goto(arm_presets[key])
                elif key == '+' or key == '=':
                    ARM_STEP = min(50, ARM_STEP + 5)
                    print(f"\r  步长: {ARM_STEP}mm                ")
                elif key == '-' or key == '_':
                    ARM_STEP = max(1, ARM_STEP - 5)
                    print(f"\r  步长: {ARM_STEP}mm                ")

        print()


if __name__ == '__main__':
    try:
        RobotConsole().run()
    except rospy.ROSInterruptException:
        pass
