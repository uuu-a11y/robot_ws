#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tty
import termios
import sys
import os
import json
import math
import re
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from arm_controller.srv import PickPlace, move
from arm_controller.msg import control
from ar_pose.srv import Track
from sensor_msgs.msg import JointState

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
COORDINATE_FILE = os.path.join(SCRIPT_DIR, "coordinates.json")
MISSION_FILE = os.path.join(SCRIPT_DIR, "mission_final.py")

STEP = 10
NAV_WAIT_TIME = 10.0
ALIGN_DIST = 0.25

DEFAULT_COORDINATES = {
    "arm_positions": {
        "safe": {"x": 150, "y": 0, "z": 100},
        "camera": {"x": 90, "y": 120, "z": 100},
        "table_place": {"x": 80, "y": -190, "z": 30},
        "buffer_front": {"x": 110, "y": 120, "z": 40},
        "buffer_back": {"x": 110, "y": 180, "z": 40},
        "table_camera": {"x": 110, "y": -170, "z": 100}
    },
    "nav_points": {
        "start": {"x": 0.0, "y": 0.0, "ar_id": 0},
        "station_5": {"x": 0.6, "y": 1.2, "ar_id": 1},
        "station_1": {"x": 2.0, "y": 2.2, "ar_id": 1}
    },
    "settings": {
        "align_dist": 0.25,
        "nav_wait_time": 10.0,
        "ar_target_1": 3,
        "ar_target_2": 9
    }
}


class CoordinateManager:
    def __init__(self):
        self.data = {}
        self.load()

    def load(self):
        if os.path.exists(COORDINATE_FILE):
            try:
                with open(COORDINATE_FILE, 'r') as f:
                    self.data = json.load(f)
                self._ensure_keys()
            except (json.JSONDecodeError, IOError):
                self.data = json.loads(json.dumps(DEFAULT_COORDINATES))
        else:
            self.data = json.loads(json.dumps(DEFAULT_COORDINATES))
            self.save()

    def _ensure_keys(self):
        for key in ['arm_positions', 'nav_points', 'settings']:
            if key not in self.data:
                self.data[key] = DEFAULT_COORDINATES.get(key, {})

    def save(self):
        try:
            with open(COORDINATE_FILE, 'w') as f:
                json.dump(self.data, f, indent=2, ensure_ascii=False)
        except IOError as e:
            print(f"\n保存坐标文件失败: {e}")

    def list_arm_coords(self):
        coords = self.data.get('arm_positions', {})
        if not coords:
            print("  (无机械臂坐标)")
            return
        print(f"  {'名称':<18} {'X':>6} {'Y':>7} {'Z':>6}")
        print(f"  {'-'*18} {'-'*6} {'-'*7} {'-'*6}")
        for name, c in coords.items():
            print(f"  {name:<18} {c['x']:>6} {c['y']:>7} {c['z']:>6}")

    def list_nav_coords(self):
        coords = self.data.get('nav_points', {})
        if not coords:
            print("  (无导航点坐标)")
            return
        print(f"  {'名称':<16} {'X':>7} {'Y':>7} {'AR_ID':>6}")
        print(f"  {'-'*16} {'-'*7} {'-'*7} {'-'*6}")
        for name, c in coords.items():
            ar_id = c.get('ar_id', 0)
            print(f"  {name:<16} {c['x']:>7.2f} {c['y']:>7.2f} {ar_id:>6}")

    def add_arm_coord(self, name, x, y, z):
        name = name.strip()
        if not name:
            return False, "名称不能为空"
        if name in self.data['arm_positions']:
            return False, f"名称 '{name}' 已存在，请先删除或重命名"
        self.data['arm_positions'][name] = {'x': x, 'y': y, 'z': z}
        self.save()
        return True, f"已添加机械臂坐标: {name}({x}, {y}, {z})"

    def delete_arm_coord(self, name):
        name = name.strip()
        if name in self.data['arm_positions']:
            del self.data['arm_positions'][name]
            self.save()
            return True, f"已删除机械臂坐标: {name}"
        return False, f"未找到机械臂坐标: {name}"

    def rename_arm_coord(self, old_name, new_name):
        old_name = old_name.strip()
        new_name = new_name.strip()
        if old_name not in self.data['arm_positions']:
            return False, f"未找到机械臂坐标: {old_name}"
        if new_name in self.data['arm_positions']:
            return False, f"目标名称 '{new_name}' 已存在"
        self.data['arm_positions'][new_name] = self.data['arm_positions'].pop(old_name)
        self.save()
        return True, f"已重命名: {old_name} -> {new_name}"

    def modify_arm_coord(self, name, x, y, z):
        name = name.strip()
        if name not in self.data['arm_positions']:
            return False, f"未找到机械臂坐标: {name}"
        self.data['arm_positions'][name] = {'x': x, 'y': y, 'z': z}
        self.save()
        return True, f"已修改 {name} -> ({x}, {y}, {z})"

    def get_arm_coord(self, name):
        return self.data.get('arm_positions', {}).get(name)

    def add_nav_coord(self, name, x, y, ar_id=0):
        name = name.strip()
        if not name:
            return False, "名称不能为空"
        if name in self.data['nav_points']:
            return False, f"名称 '{name}' 已存在，请先删除或重命名"
        self.data['nav_points'][name] = {'x': x, 'y': y, 'ar_id': ar_id}
        self.save()
        return True, f"已添加导航点: {name}({x:.2f}, {y:.2f}) AR-{ar_id}"

    def delete_nav_coord(self, name):
        name = name.strip()
        if name in self.data['nav_points']:
            del self.data['nav_points'][name]
            self.save()
            return True, f"已删除导航点: {name}"
        return False, f"未找到导航点: {name}"

    def rename_nav_coord(self, old_name, new_name):
        old_name = old_name.strip()
        new_name = new_name.strip()
        if old_name not in self.data['nav_points']:
            return False, f"未找到导航点: {old_name}"
        if new_name in self.data['nav_points']:
            return False, f"目标名称 '{new_name}' 已存在"
        self.data['nav_points'][new_name] = self.data['nav_points'].pop(old_name)
        self.save()
        return True, f"已重命名: {old_name} -> {new_name}"

    def modify_nav_coord(self, name, x, y, ar_id):
        name = name.strip()
        if name not in self.data['nav_points']:
            return False, f"未找到导航点: {name}"
        self.data['nav_points'][name] = {'x': x, 'y': y, 'ar_id': ar_id}
        self.save()
        return True, f"已修改 {name} -> ({x:.2f}, {y:.2f}) AR-{ar_id}"

    def get_nav_coord(self, name):
        return self.data.get('nav_points', {}).get(name)

    def get_all_arm_names(self):
        return list(self.data.get('arm_positions', {}).keys())

    def get_all_nav_names(self):
        return list(self.data.get('nav_points', {}).keys())

    def sync_to_mission(self):
        if not os.path.exists(MISSION_FILE):
            return False, f"找不到 mission_final.py: {MISSION_FILE}"

        try:
            with open(MISSION_FILE, 'r') as f:
                content = f.read()

            settings = self.data.get('settings', {})

            nav_lines = []
            nav_lines.append("# ================= 导航坐标 (单位: m) =================")
            for name, c in self.data.get('nav_points', {}).items():
                var_name = name.upper()
                ar_id = c.get('ar_id', 0)
                nav_lines.append(f"{var_name}_X = {c['x']}")
                nav_lines.append(f"{var_name}_Y = {c['y']}")
                if ar_id > 0:
                    nav_lines.append(f"{var_name}_AR_ID = {ar_id}")
                nav_lines.append("")

            arm_lines = []
            arm_lines.append("# ================= 机械臂坐标 (单位: mm) =================")
            for name, c in self.data.get('arm_positions', {}).items():
                var_name = name.upper()
                arm_lines.append(f"{var_name}_X = {c['x']}")
                arm_lines.append(f"{var_name}_Y = {c['y']}")
                arm_lines.append(f"{var_name}_Z = {c['z']}")
                arm_lines.append("")

            other_lines = []
            other_lines.append("# ================= AR物块ID =================")
            other_lines.append(f"AR_TARGET_1 = {settings.get('ar_target_1', 3)}")
            other_lines.append(f"AR_TARGET_2 = {settings.get('ar_target_2', 9)}")
            other_lines.append("")
            other_lines.append("# ================= 对准距离 =================")
            other_lines.append(f"ALIGN_DIST = {settings.get('align_dist', 0.25)}")
            other_lines.append("")
            other_lines.append("# ================= 导航等待时间 =================")
            other_lines.append(f"NAV_WAIT_TIME = {settings.get('nav_wait_time', 10.0)}")

            pattern_nav = r'# ================= 导航坐标.*?\n(.*?)(?=# ================= 机械臂坐标)'
            pattern_arm = r'# ================= 机械臂坐标.*?\n(.*?)(?=# ================= AR物块ID)'
            pattern_other = r'# ================= AR物块ID.*\n(.*?)(?=class FinalMission)'

            new_content = content

            match_nav = re.search(pattern_nav, new_content, re.DOTALL)
            if match_nav:
                nav_block = '\n'.join(nav_lines) + '\n'
                new_content = new_content[:match_nav.start()] + nav_block + new_content[match_nav.end():]

            match_arm = re.search(pattern_arm, new_content, re.DOTALL)
            if match_arm:
                arm_block = '\n'.join(arm_lines) + '\n'
                new_content = new_content[:match_arm.start()] + arm_block + new_content[match_arm.end():]

            match_other = re.search(pattern_other, new_content, re.DOTALL)
            if match_other:
                other_block = '\n'.join(other_lines) + '\n\n'
                new_content = new_content[:match_other.start()] + other_block + new_content[match_other.end():]

            for name, c in self.data.get('arm_positions', {}).items():
                var_name = name.upper()
                for axis in ['X', 'Y', 'Z']:
                    old_var = f"{var_name[0:-2]}_{axis}" if var_name.endswith('_X') or var_name.endswith('_Y') or var_name.endswith('_Z') else None
                    if old_var:
                        new_var = f"{var_name}_{axis}"
                        new_content = new_content.replace(old_var, new_var)

            for name, c in self.data.get('nav_points', {}).items():
                var_name = name.upper()
                for axis in ['X', 'Y']:
                    old_var = f"{var_name[0:-2]}_{axis}" if (var_name.endswith('_X') or var_name.endswith('_Y')) and len(var_name) > 2 else None
                    if old_var:
                        new_var = f"{var_name}_{axis}"
                        new_content = new_content.replace(old_var, new_var)

            with open(MISSION_FILE, 'w') as f:
                f.write(new_content)

            return True, f"已同步到 mission_final.py ({len(self.data.get('nav_points',{}))}个导航点, {len(self.data.get('arm_positions',{}))}个机械臂坐标)"

        except Exception as e:
            return False, f"同步失败: {e}"


class UnifiedDebug:
    def __init__(self):
        rospy.init_node('unified_debug', anonymous=True)

        self.coord_mgr = CoordinateManager()

        self.x = 150.0
        self.y = 0.0
        self.z = 100.0
        self.joint_angles = [0.0, 0.0, 0.0]

        print("=" * 60)
        print("   统一调试控制台 (键盘+菜单)")
        print("=" * 60)
        print("\n>>> 正在连接服务...")
        try:
            rospy.wait_for_service('/goto_position', timeout=10)
            rospy.wait_for_service('/swiftpro/on', timeout=10)
            rospy.wait_for_service('/swiftpro/off', timeout=10)
            rospy.wait_for_service('/move_base/make_plan', timeout=10)
            rospy.wait_for_service('/ar_track', timeout=10)
            rospy.wait_for_service('/pick_ar', timeout=10)
        except rospy.ROSException:
            print(">>> 连接超时！请检查节点是否启动。")
            exit(1)

        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.srv_ar_track = rospy.ServiceProxy('/ar_track', Track)
        self.srv_pick_ar = rospy.ServiceProxy('/pick_ar', PickPlace)

        rospy.Subscriber('/swiftpro/joint_states', JointState, self.joint_callback)

        print(">>> 所有服务连接成功！\n")

    def joint_callback(self, msg):
        if len(msg.position) >= 3:
            self.joint_angles[0] = math.degrees(msg.position[0])
            self.joint_angles[1] = math.degrees(msg.position[1])
            self.joint_angles[2] = math.degrees(msg.position[2])

    def move_arm(self, dx=0, dy=0, dz=0, absolute=None):
        if absolute:
            self.x, self.y, self.z = absolute
        else:
            self.x = max(0, min(280, self.x + dx))
            self.y = max(-278, min(278, self.y + dy))
            self.z = max(0, min(130, self.z + dz))

        p = control()
        p.position.x = float(self.x)
        p.position.y = float(self.y)
        p.position.z = float(self.z)
        p.roll = 0.0
        p.pitch = 0.0
        p.yaw = 0.0

        try:
            response = self.srv_arm_move(pose=p)
            return response.success
        except Exception as e:
            print(f"\n机械臂错误: {e}")
            return False

    def go_to_arm_coord(self, name):
        coord = self.coord_mgr.get_arm_coord(name)
        if coord:
            self.move_arm(absolute=(coord['x'], coord['y'], coord['z']))
            return True
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

    def navigate_to(self, x, y):
        print(f"\n[导航] 目标: ({x:.2f}, {y:.2f})")
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        self.pub_goal.publish(goal)
        settings = self.coord_mgr.data.get('settings', {})
        wait = settings.get('nav_wait_time', NAV_WAIT_TIME)
        print(f"[导航] 等待 {wait} 秒...")
        rospy.sleep(wait)
        print(f"[导航] 完成")

    def go_to_nav_point(self, name):
        coord = self.coord_mgr.get_nav_coord(name)
        if coord:
            self.navigate_to(coord['x'], coord['y'])
            if coord.get('ar_id', 0) > 0:
                self.ar_align(coord['ar_id'])
            return True
        return False

    def ar_align(self, ar_id):
        dist = self.coord_mgr.data.get('settings', {}).get('align_dist', ALIGN_DIST)
        print(f"\n[AR对准] AR-{ar_id} 距离{dist}m")
        try:
            response = self.srv_ar_track(ar_id=int(ar_id), goal_dist=dist)
            if response.success:
                print(f"[AR对准] AR-{ar_id} 成功")
            else:
                print(f"[AR对准] AR-{ar_id} 失败: {response.message}")
        except Exception as e:
            print(f"[AR对准] 错误: {e}")
        rospy.sleep(1)

    def visual_grab(self, ar_id):
        table = self.coord_mgr.get_arm_coord('table_place')
        if not table:
            table = {'x': 80, 'y': -190, 'z': 30}

        target = control()
        target.position.x = float(table['x'])
        target.position.y = float(table['y'])
        target.position.z = float(table['z'])
        target.roll = 0.0
        target.pitch = 0.0
        target.yaw = 0.0

        print(f"\n[视觉抓取] AR-{ar_id}")
        try:
            response = self.srv_pick_ar(number=ar_id, mode=0, pose=target)
            if response.success:
                print(f"[视觉抓取] AR-{ar_id} 成功")
            else:
                print(f"[视觉抓取] 失败: {response.message}")
        except Exception as e:
            print(f"[视觉抓取] 错误: {e}")
        rospy.sleep(1)

    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def command_mode(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, termios.tcgetattr(fd))
            os.system('clear' if os.name == 'posix' else 'cls')

            print("\n" + "=" * 50)
            print("   命令模式")
            print("=" * 50)
            print("""
  a1  : 查看机械臂坐标
  a2  : 添加机械臂坐标
  a3  : 删除机械臂坐标
  a4  : 重命名机械臂坐标
  a5  : 修改机械臂坐标

  n1  : 查看导航点
  n2  : 添加导航点
  n3  : 删除导航点
  n4  : 重命名导航点
  n5  : 修改导航点
  n6  : 导航到指定点
  n7  : AR码对准
  n8  : 视觉抓取

  s   : 同步到 mission_final.py
  q   : 退出命令模式
""")
            print("-" * 50)

            cmd = input("输入命令: ").strip().lower()

            if cmd == 'a1':
                print("\n--- 机械臂坐标列表 ---")
                self.coord_mgr.list_arm_coords()

            elif cmd == 'a2':
                print("\n--- 添加机械臂坐标 ---")
                name = input("  名称: ").strip()
                if not name:
                    print("  取消")
                else:
                    try:
                        x = int(input("  X (mm): ").strip())
                        y = int(input("  Y (mm): ").strip())
                        z = int(input("  Z (mm): ").strip())
                        ok, msg = self.coord_mgr.add_arm_coord(name, x, y, z)
                        print(f"  {msg}")
                    except ValueError:
                        print("  输入无效")

            elif cmd == 'a3':
                print("\n--- 删除机械臂坐标 ---")
                self.coord_mgr.list_arm_coords()
                name = input("  要删除的名称: ").strip()
                ok, msg = self.coord_mgr.delete_arm_coord(name)
                print(f"  {msg}")

            elif cmd == 'a4':
                print("\n--- 重命名机械臂坐标 ---")
                self.coord_mgr.list_arm_coords()
                old_name = input("  原名称: ").strip()
                new_name = input("  新名称: ").strip()
                ok, msg = self.coord_mgr.rename_arm_coord(old_name, new_name)
                print(f"  {msg}")

            elif cmd == 'a5':
                print("\n--- 修改机械臂坐标 ---")
                self.coord_mgr.list_arm_coords()
                name = input("  选择名称: ").strip()
                if not name or not self.coord_mgr.get_arm_coord(name):
                    print("  未找到该坐标")
                else:
                    old = self.coord_mgr.get_arm_coord(name)
                    print(f"  原值: ({old['x']}, {old['y']}, {old['z']})")
                    try:
                        x = int(input("  X (mm): ").strip())
                        y = int(input("  Y (mm): ").strip())
                        z = int(input("  Z (mm): ").strip())
                        ok, msg = self.coord_mgr.modify_arm_coord(name, x, y, z)
                        print(f"  {msg}")
                    except ValueError:
                        print("  输入无效")

            elif cmd == 'n1':
                print("\n--- 导航点列表 ---")
                self.coord_mgr.list_nav_coords()

            elif cmd == 'n2':
                print("\n--- 添加导航点 ---")
                name = input("  名称: ").strip()
                if not name:
                    print("  取消")
                else:
                    try:
                        x = float(input("  X (m): ").strip())
                        y = float(input("  Y (m): ").strip())
                        ar_input = input("  AR ID (0=无, 默认1): ").strip()
                        ar_id = int(ar_input) if ar_input else 1
                        ok, msg = self.coord_mgr.add_nav_coord(name, x, y, ar_id)
                        print(f"  {msg}")
                    except ValueError:
                        print("  输入无效")

            elif cmd == 'n3':
                print("\n--- 删除导航点 ---")
                self.coord_mgr.list_nav_coords()
                name = input("  要删除的名称: ").strip()
                ok, msg = self.coord_mgr.delete_nav_coord(name)
                print(f"  {msg}")

            elif cmd == 'n4':
                print("\n--- 重命名导航点 ---")
                self.coord_mgr.list_nav_coords()
                old_name = input("  原名称: ").strip()
                new_name = input("  新名称: ").strip()
                ok, msg = self.coord_mgr.rename_nav_coord(old_name, new_name)
                print(f"  {msg}")

            elif cmd == 'n5':
                print("\n--- 修改导航点 ---")
                self.coord_mgr.list_nav_coords()
                name = input("  选择名称: ").strip()
                if not name or not self.coord_mgr.get_nav_coord(name):
                    print("  未找到该导航点")
                else:
                    old = self.coord_mgr.get_nav_coord(name)
                    print(f"  原值: ({old['x']:.2f}, {old['y']:.2f}) AR-{old.get('ar_id',0)}")
                    try:
                        x = float(input("  X (m): ").strip())
                        y = float(input("  Y (m): ").strip())
                        ar_input = input("  AR ID (0=无): ").strip()
                        ar_id = int(ar_input) if ar_input else 0
                        ok, msg = self.coord_mgr.modify_nav_coord(name, x, y, ar_id)
                        print(f"  {msg}")
                    except ValueError:
                        print("  输入无效")

            elif cmd == 'n6':
                print("\n--- 导航到指定点 ---")
                self.coord_mgr.list_nav_coords()
                name = input("  选择名称或直接输入坐标(X,Y): ").strip()
                coord = self.coord_mgr.get_nav_coord(name)
                if coord:
                    print(f"  正在导航到 {name}...")
                    self.go_to_nav_point(name)
                else:
                    if ',' in name:
                        parts = name.replace(' ', '').split(',')
                        if len(parts) == 2:
                            try:
                                self.navigate_to(float(parts[0]), float(parts[1]))
                            except ValueError:
                                print("  输入无效")
                        else:
                            print("  格式应为 X,Y")
                    else:
                        print("  未找到该导航点")

            elif cmd == 'n7':
                print("\n--- AR码对准 ---")
                ar_id = input("  AR ID: ").strip()
                try:
                    self.ar_align(int(ar_id))
                except ValueError:
                    print("  输入无效")

            elif cmd == 'n8':
                print("\n--- 视觉抓取 ---")
                ar_id = input("  AR ID (3或9): ").strip()
                try:
                    self.visual_grab(int(ar_id))
                except ValueError:
                    print("  输入无效")

            elif cmd == 's':
                print("\n--- 同步到 mission_final.py ---")
                ok, msg = self.coord_mgr.sync_to_mission()
                print(f"  {msg}")

            elif cmd == 'q':
                print("\n  返回键盘控制模式...")
                return

            else:
                print(f"  未知命令: {cmd}")

            input("\n按 Enter 继续...")

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def show_quick_help(self):
        arm_names = self.coord_mgr.get_all_arm_names()
        nav_names = self.coord_mgr.get_all_nav_names()

        print("=" * 50)
        print("   快捷键")
        print("=" * 50)
        print("""
  W/S/A/D/Q/E  : 机械臂移动 (XYZ)
  空格/F       : 气泵 开/关
  R           : 归位
  P           : 视觉抓取 AR-3
  O           : 视觉抓取 AR-9

  === 机械臂位置 (数字键) ===""")

        for i, name in enumerate(arm_names[:9]):
            coord = self.coord_mgr.get_arm_coord(name)
            if coord:
                print(f"  {i+1}          : {name:<14} ({coord['x']},{coord['y']},{coord['z']})")

        print("""
  === 导航点 (J+数字) ===""")

        for i, name in enumerate(nav_names[:9]):
            coord = self.coord_mgr.get_nav_coord(name)
            if coord:
                ar_info = f" AR{coord['ar_id']}" if coord.get('ar_id', 0) > 0 else ""
                print(f"  J{i+1}        : {name:<12} ({coord['x']:.1f},{coord['y']:.1f}){ar_info}")

        print(f"""
  M           : 命令模式
  H           : 显示此帮助
  Ctrl+C      : 退出
""")

    def run(self):
        self.show_quick_help()
        print("=" * 50)
        print("  就绪 - 等待输入...")

        while not rospy.is_shutdown():
            status = f"X:{self.x:3.0f} Y:{self.y:4.0f} Z:{self.z:3.0f} | J1:{self.joint_angles[0]:6.1f} J2:{self.joint_angles[1]:6.1f} J3:{self.joint_angles[2]:6.1f}"
            print(f"\r{status}    ", end="", flush=True)

            key = self.read_key()

            if key == '\x03':
                print("\n退出")
                break

            elif key in ['w', 'W']:
                self.move_arm(STEP, 0, 0)

            elif key in ['s', 'S']:
                self.move_arm(-STEP, 0, 0)

            elif key in ['a', 'A']:
                self.move_arm(0, -STEP, 0)

            elif key in ['d', 'D']:
                self.move_arm(0, STEP, 0)

            elif key in ['q', 'Q']:
                self.move_arm(0, 0, STEP)

            elif key in ['e', 'E']:
                self.move_arm(0, 0, -STEP)

            elif key == ' ':
                self.pump_on()
                print("\r气泵 ON                                                                  ", end="", flush=True)

            elif key in ['f', 'F']:
                self.pump_off()
                print("\r气泵 OFF                                                                 ", end="", flush=True)

            elif key in ['r', 'R']:
                self.move_arm(absolute=(150, 0, 100))
                print("\r归位 (150,0,100)                                                           ", end="", flush=True)

            elif key in ['p', 'P']:
                self.visual_grab(3)

            elif key in ['o', 'O']:
                self.visual_grab(9)

            elif key in ['m', 'M']:
                self.command_mode()
                os.system('clear' if os.name == 'posix' else 'cls')
                self.show_quick_help()
                print("  命令模式已退出，可以继续操作\n")

            elif key in ['h', 'H', '?']:
                self.show_quick_help()

            elif key.isdigit() and key != '0':
                idx = int(key) - 1
                arm_names = self.coord_mgr.get_all_arm_names()
                if idx < len(arm_names):
                    name = arm_names[idx]
                    if self.go_to_arm_coord(name):
                        coord = self.coord_mgr.get_arm_coord(name)
                        print(f"\r{name} ({coord['x']},{coord['y']},{coord['z']})", end="", flush=True)

            elif key in ['j', 'J']:
                key2 = self.read_key()
                if key2.isdigit() and key2 != '0':
                    idx = int(key2) - 1
                    nav_names = self.coord_mgr.get_all_nav_names()
                    if idx < len(nav_names):
                        name = nav_names[idx]
                        print(f"\r导航到 {name}...", end="", flush=True)
                        self.go_to_nav_point(name)

if __name__ == "__main__":
    try:
        UnifiedDebug().run()
    except rospy.ROSInterruptException:
        pass
