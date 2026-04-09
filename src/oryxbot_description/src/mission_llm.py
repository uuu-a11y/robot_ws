#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import re
import requests
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from arm_controller.srv import PickPlace, move
from arm_controller.msg import control
from ar_pose.srv import Track
from relative_move.srv import SetRelativeMove
from geometry_msgs.msg import Pose2D
from actionlib_msgs.msg import GoalStatusArray, GoalID

ZHIPU_API_KEY = "50713e5610684389ab3c0ebeacc88ce8.0ntImscaIx3NwTee"
MODEL_TYPE = "glm-4-flash"

NAVI_POINTS = {
    "start": {"x": 0.0, "y": 0.0, "ar_id": 0, "angle": 0},
    "station_1": {"x": 2.0, "y": 2.2, "ar_id": 1, "angle": 0},
    "station_2": {"x": 2.2, "y": 1.2, "ar_id": 1, "angle": 0},
    "station_3": {"x": 2.2, "y": 0.2, "ar_id": 1, "angle": 0},
    "station_4": {"x": 0.6, "y": 2.2, "ar_id": 1, "angle": 0},
    "station_5": {"x": 0.6, "y": 1.2, "ar_id": 1, "angle": 0},
    "charging_station": {"x": 0.4, "y": 2.0, "ar_id": 0, "angle": 90}
}

ARM_POSITIONS = {
    "safe": {"x": 150, "y": 0, "z": 100},
    "camera": {"x": 90, "y": 120, "z": 100, "name": "摄像头位置", "desc": "面向储物槽"},
    "table_camera": {"x": 110, "y": -170, "z": 100, "name": "台面相机位", "desc": "面向台面"},
    "table_place": {"x": 60, "y": -230, "z": 30, "name": "台面放置位"},
    "buffer_front": {"x": 110, "y": 120, "z": 40, "name": "前储物槽"},
    "buffer_back": {"x": 110, "y": 180, "z": 40, "name": "后储物槽"}
}

SYSTEM_PROMPT = """你是机器人任务规划助手。

【导航点】
- start: 起点
- station_1~5: 1~5号工位

【动作类型 - 必须严格遵守】
1. car_to_table: 物块从车上放到台面
   触发词: "运X到Y"、"送X到Y"、"运送到"
   
2. table_to_car: 物块从台面取到车上
   触发词: "从X取Y"、"取到车上"、"取到小车上"

3. nav_only: 仅导航
   触发词: "回起点"、"去X"、"导航到"

【拆解规则】
- "从A把X运到B" = table_to_car(A,X) + car_to_table(B,X)
- "把X从A运到B" = table_to_car(A,X) + car_to_table(B,X)
- "运X到Y" = car_to_table(Y,X)  【物块默认在车上】
- "送X到Y" = car_to_table(Y,X)  【物块默认在车上】
- "从X取Y" = table_to_car(X,Y)
- "取Y到车上" = table_to_car(X,Y) 【X由上下文推断】
- "回起点" = nav_only(start)

【关键】
- nav_only 不需要 ar_id！
- car_to_table 和 table_to_car 必须有 nav 和 ar_id！
- 仔细分析"从...运..."和"运...到..."的区别！

【示例】
用户: "把5号物块从1号运到3号"
分析: 从1号取(table_to_car) + 运到3号(car_to_table)
[
  {"action": "table_to_car", "nav": "station_1", "ar_id": 5},
  {"action": "car_to_table", "nav": "station_3", "ar_id": 5}
]

用户: "把2号物块运到1号"
分析: 运到1号(car_to_table)，物块默认在车上
[
  {"action": "car_to_table", "nav": "station_1", "ar_id": 2}
]

用户: "回起始点"
[
  {"action": "nav_only", "nav": "start"}
]

【返回格式】
只返回JSON数组！"""

class MissionLLM:
    def __init__(self):
        rospy.init_node('mission_llm', anonymous=True)
        
        print("=" * 60)
        print("   LLM任务规划节点")
        print("=" * 60)
        
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_cancel_goal = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        
        print(">>> 等待ROS服务...")
        try:
            rospy.wait_for_service('/ar_track', timeout=15)
            rospy.wait_for_service('/goto_position', timeout=15)
            rospy.wait_for_service('/swiftpro/on', timeout=15)
            rospy.wait_for_service('/swiftpro/off', timeout=15)
            rospy.wait_for_service('/relative_move', timeout=15)
        except rospy.ROSException:
            print(">>> 连接超时！请检查节点是否启动")
            exit(1)
        
        self.srv_ar_track = rospy.ServiceProxy('/ar_track', Track)
        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pick_ar = rospy.ServiceProxy('/pick_ar', PickPlace)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)
        self.srv_relative_move = rospy.ServiceProxy('/relative_move', SetRelativeMove)
        
        rospy.sleep(1)
        print(">>> 连接成功！\n")
        
        self.current_target = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal_active = False
        self.goal_reached = False
        self.goal_aborted = False
        self.sub_goal_status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.on_goal_status)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.on_odom)
        
        self.run_interactive()
    
    def on_odom(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_qx = msg.pose.pose.orientation.x
        self.robot_qy = msg.pose.pose.orientation.y
        self.robot_qz = msg.pose.pose.orientation.z
        self.robot_qw = msg.pose.pose.orientation.w
    
    def on_goal_status(self, msg):
        if not msg.status_list:
            return
        
        # Only look at the LAST status entry (the current active goal)
        last_status = msg.status_list[-1]
        if last_status.status == 3:
            self.goal_reached = True
        elif last_status.status in [4, 5, 8]:
            self.goal_aborted = True
    
    def call_llm(self, user_message):
        url = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
        headers = {
            "Authorization": f"Bearer {ZHIPU_API_KEY}",
            "Content-Type": "application/json"
        }
        
        payload = {
            "model": MODEL_TYPE,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_message}
            ]
        }
        
        try:
            response = requests.post(url, headers=headers, json=payload, timeout=60)
            result = response.json()
            
            if 'choices' in result and result['choices']:
                return result['choices'][0]['message']['content']
            else:
                print(f"[调试] 响应: {result}")
                return None
        except Exception as e:
            print(f"[错误] API调用失败: {e}")
            return None
    
    def parse_response(self, response_text):
        if not response_text:
            return None
        
        tasks = []
        
        try:
            json_blocks = re.findall(r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}', response_text, re.DOTALL)
            
            for block in json_blocks:
                try:
                    task = json.loads(block)
                    if 'action' in task:
                        tasks.append(task)
                except:
                    continue
            
            if tasks:
                return tasks if len(tasks) > 1 else tasks[0]
        except:
            pass
        
        return None
    
    def navigate_to(self, nav_point):
        import math
        if nav_point not in NAVI_POINTS:
            rospy.logerr(f"未知导航点: {nav_point}")
            return False
        
        point = NAVI_POINTS[nav_point]
        print(f"[导航] 目标: {nav_point} ({point['x']}, {point['y']})")
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = point['x']
        goal.pose.position.y = point['y']
        goal.pose.position.z = 0.0
        
        angle = point.get('angle')
        if angle is not None:
            yaw_rad = math.radians(angle)
            goal.pose.orientation.z = math.sin(yaw_rad / 2)
            goal.pose.orientation.w = math.cos(yaw_rad / 2)
        else:
            # 保持当前车体朝向，避免导航时旋转
            qz = self.robot_qz
            qw = self.robot_qw
            # 如果odom还没收到数据(全0是无效四元数)，默认朝东
            if abs(qz) < 1e-6 and abs(qw) < 1e-6:
                qz = 0.0
                qw = 1.0
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        
        # Reset status flags BEFORE sending goal
        self.goal_reached = False
        self.goal_aborted = False
        
        self.pub_goal.publish(goal)
        print(f"[导航] 目标已发送，等待 move_base 完成...")
        
        # Primary: wait for move_base status=3 (SUCCEEDED)
        # Fallback: if move_base aborts but we're close enough (< 0.15m), accept it
        rate = rospy.Rate(2)
        timeout = 120
        min_wait_time = 5
        waited = 0
        nav_done = False
        for i in range(int(timeout * 2)):
            waited += 0.5
            dist = ((self.robot_x - point['x'])**2 + (self.robot_y - point['y'])**2)**0.5
            
            # Primary: move_base reported success
            if self.goal_reached:
                print(f"[导航] move_base 报告到达! 位置: ({self.robot_x:.2f}, {self.robot_y:.2f}), 距离: {dist:.2f}m")
                self.pub_cancel_goal.publish(GoalID())
                rospy.sleep(1.5)
                nav_done = True
                break
            
            # Fallback: move_base aborted but we're close enough
            if self.goal_aborted and waited > min_wait_time:
                if dist < 0.15:
                    print(f"[导航] move_base 中止但已足够近 (距离: {dist:.2f}m)，继续执行")
                else:
                    print(f"[导航] move_base 中止，距离: {dist:.2f}m，继续执行")
                self.pub_cancel_goal.publish(GoalID())
                rospy.sleep(1.5)
                nav_done = True
                break
            
            # Print progress every 2 seconds
            if i % 4 == 0:
                print(f"[导航] 位置: ({self.robot_x:.2f}, {self.robot_y:.2f}), 距离: {dist:.2f}m")
            
            rate.sleep()
        
        if not nav_done:
            print(f"[导航] 等待超时({timeout}s)，继续执行")
            self.pub_cancel_goal.publish(GoalID())
            rospy.sleep(1.5)
        
        # 精确补位：如果距离目标 > 0.05m，用 RelativeMove 精确移动到目标位置
        dist = ((self.robot_x - point['x'])**2 + (self.robot_y - point['y'])**2)**0.5
        if dist > 0.03:
            dx = point['x'] - self.robot_x
            dy = point['y'] - self.robot_y
            print(f"[导航] 精确补位: 偏差 {dist:.3f}m, 移动 dx={dx:.3f} dy={dy:.3f}")
            try:
                goal_rm = Pose2D(dx, dy, 0.0)
                resp = self.srv_relative_move(goal_rm, "odom", False, False)
                if resp.success:
                    rospy.sleep(0.5)
                    dist_new = ((self.robot_x - point['x'])**2 + (self.robot_y - point['y'])**2)**0.5
                    print(f"[导航] 补位完成, 剩余偏差: {dist_new:.3f}m")
                else:
                    print(f"[导航] 补位失败: {resp.message}")
            except Exception as e:
                print(f"[导航] 补位异常: {e}")
        
        # 角度补位：如果有指定朝向，修正角度偏差
        angle = point.get('angle')
        if angle is not None:
            import math
            target_yaw = math.radians(angle)
            # 从四元数计算当前朝向
            robot_yaw = math.atan2(2.0 * (self.robot_qw * self.robot_qz), 
                                    1.0 - 2.0 * (self.robot_qz * self.robot_qz))
            yaw_err = target_yaw - robot_yaw
            # 归一化到 [-pi, pi]
            while yaw_err > math.pi: yaw_err -= 2 * math.pi
            while yaw_err < -math.pi: yaw_err += 2 * math.pi
            if abs(yaw_err) > 0.05:  # 超过 ~3° 才修正
                print(f"[导航] 角度补位: 当前 {math.degrees(robot_yaw):.1f}°, 目标 {angle}°, 偏差 {math.degrees(yaw_err):.1f}°")
                try:
                    goal_rm = Pose2D(0.0, 0.0, yaw_err)
                    resp = self.srv_relative_move(goal_rm, "odom", False, False)
                    if resp.success:
                        rospy.sleep(0.5)
                        print(f"[导航] 角度补位完成")
                    else:
                        print(f"[导航] 角度补位失败: {resp.message}")
                except Exception as e:
                    print(f"[导航] 角度补位异常: {e}")
        
        return True
    
    def ar_align(self, ar_id):
        if ar_id is None or ar_id <= 0:
            print(f"[AR] 跳过对准（ID={ar_id}）")
            return True
        
        print(f"[AR] 对准 AR-{ar_id}")
        try:
            response = self.srv_ar_track(ar_id=int(ar_id), goal_dist=0.25)
            if response.success:
                print(f"[AR] 对准成功，等待小车稳定...")
                rospy.sleep(1.5)  # 等待小车完全停止稳定
            else:
                print(f"[AR] 对准失败: {response.message}")
                return False
        except Exception as e:
            print(f"[AR] 错误: {e}")
            return False
        return True
    
    def move_arm(self, position):
        """Move arm to named position or (x, y, z) tuple"""
        if isinstance(position, tuple):
            pos = {'x': position[0], 'y': position[1], 'z': position[2]}
            name_str = f"({pos['x']}, {pos['y']}, {pos['z']})"
        elif position in ARM_POSITIONS:
            pos = ARM_POSITIONS[position]
            name_str = f"{position} ({pos['x']}, {pos['y']}, {pos['z']})"
        else:
            rospy.logerr(f"未知机械臂位置: {position}")
            return False
        
        print(f"[机械臂] 移动到 {name_str}")
        
        if not (0 <= pos['x'] <= 280 and -278 <= pos['y'] <= 278 and 0 <= pos['z'] <= 130):
            print(f"[错误] 坐标超出范围! X:0~280, Y:-278~278, Z:0~130")
            return False
        
        try:
            p = control()
            p.position.x = float(pos['x'])
            p.position.y = float(pos['y'])
            p.position.z = float(pos['z'])
            p.roll = 0.0
            p.pitch = 0.0
            p.yaw = 0.0
            self.srv_arm_move(pose=p)
            rospy.sleep(0.2)
            return True
        except Exception as e:
            print(f"[机械臂] 错误: {e}")
            # Try to reconnect if service is unavailable
            if "unavailable" in str(e):
                print("[机械臂] 尝试重新连接 goto_position 服务...")
                try:
                    rospy.wait_for_service('/goto_position', timeout=5)
                    self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
                    self.srv_arm_move(pose=p)
                    rospy.sleep(0.2)
                    print("[机械臂] 重连成功")
                    return True
                except:
                    print("[机械臂] 重连失败")
            return False
    
    def pump_on(self):
        print("[气泵] 开启")
        try:
            self.srv_pump_on()
            rospy.sleep(0.2)
        except:
            pass
    
    def visual_grab(self, ar_id, pose_name):
        """Returns True on success, False on failure"""
        print(f"[视觉抓取] 识别并抓取 AR-{ar_id}")
        
        # Wait for AR marker TF frame to stabilize (need consistent detection)
        import tf
        frame = f"ar_marker_{ar_id}"
        try:
            listener = tf.TransformListener()
            if listener.frameExists(frame):
                print(f"[视觉抓取] TF帧 {frame} 已存在")
                # 等待 TF 数据稳定（连续 3 次成功查询）
                stable_count = 0
                for i in range(10):
                    try:
                        listener.lookupTransform("Base", frame, rospy.Time(0))
                        stable_count += 1
                        if stable_count >= 3:
                            break
                    except:
                        stable_count = 0
                    rospy.sleep(0.2)
                if stable_count >= 3:
                    print(f"[视觉抓取] AR-{ar_id} 定位已稳定")
                else:
                    print(f"[视觉抓取] AR-{ar_id} 定位不太稳定，仍然尝试抓取")
            else:
                print(f"[视觉抓取] 等待 TF帧 {frame} 出现...")
                try:
                    listener.waitForTransform("Base", frame, rospy.Time(0), rospy.Duration(5.0))
                    print(f"[视觉抓取] TF帧 {frame} 已出现，等待定位稳定...")
                    rospy.sleep(1.0)  # 额外等待定位收敛
                except tf.Exception:
                    print(f"[视觉抓取] TF帧 {frame} 等待超时，摄像头可能看不到 AR-{ar_id}")
        except Exception as e:
            print(f"[视觉抓取] TF检查异常: {e}（继续尝试抓取）")
        
        pos = ARM_POSITIONS[pose_name]
        print(f"[视觉抓取] 放置位置: ({pos['x']}, {pos['y']}, {pos['z']})")
        
        if not (0 <= pos['x'] <= 280 and -278 <= pos['y'] <= 278 and 0 <= pos['z'] <= 130):
            print(f"[错误] 坐标超出机械臂范围!")
            print(f"[范围] X: 0~280, Y: -278~278, Z: 0~130")
            return False
        
        target = control()
        target.position.x = pos['x']
        target.position.y = pos['y']
        target.position.z = pos['z']
        target.roll = 0.0
        target.pitch = 0.0
        target.yaw = 0.0
        
        try:
            response = self.srv_pick_ar(number=ar_id, mode=1, pose=target)
            rospy.sleep(0.5)
            if response.success:
                print(f"[视觉抓取] 抓取成功")
                return True
            else:
                print(f"[视觉抓取] 失败: {response.message}")
                return False
        except Exception as e:
            print(f"[视觉抓取] 错误: {e}")
            return False
    
    def pump_off(self):
        print("[气泵] 关闭")
        try:
            self.srv_pump_off()
            rospy.sleep(0.2)
        except:
            pass
    
    def execute_task(self, task):
        action = task.get('action', '')
        nav = task.get('nav', '')
        ar_id = task.get('ar_id', 0)
        buffer_pos = task.get('buffer', 'buffer_front')
        
        align_id = NAVI_POINTS.get(nav, {}).get('ar_id', 1) if nav else 1
        max_grab_retries = 2  # How many times to re-align + re-grab after initial failure
        
        if action == 'car_to_table':
            print(f"\n[执行] 车→台面: AR-{ar_id} 到 {nav}")
            
            if nav:
                self.navigate_to(nav)
                self.ar_align(align_id)
            
            # Grab with retry
            grab_ok = False
            for retry in range(max_grab_retries + 1):
                if retry > 0:
                    print(f"\n[重试] 第{retry}次重试抓取 AR-{ar_id}")
                    self.move_arm("safe")
                    rospy.sleep(1.0)
                    if not self.ar_align(align_id):
                        print(f"[重试] AR对齐失败，放弃")
                        break
                
                self.move_arm("camera")
                rospy.sleep(1.5)  # Wait for camera to detect AR marker
                grab_ok = self.visual_grab(ar_id, "table_place")
                if grab_ok:
                    break
                print(f"[重试] 抓取未成功，准备重试...")
            
            if not grab_ok:
                print(f"[警告] 抓取 AR-{ar_id} 最终失败，继续执行下一步")
            
            self.move_arm("safe")
            print("[完成]")
        
        elif action == 'table_to_car':
            print(f"\n[执行] 台面→车: {nav} AR-{ar_id}")
            
            if nav:
                self.navigate_to(nav)
                self.ar_align(align_id)
            
            # Grab with retry
            grab_ok = False
            for retry in range(max_grab_retries + 1):
                if retry > 0:
                    print(f"\n[重试] 第{retry}次重试抓取 AR-{ar_id}")
                    self.move_arm("safe")
                    rospy.sleep(1.0)
                    if not self.ar_align(align_id):
                        print(f"[重试] AR对齐失败，放弃")
                        break
                
                self.move_arm("table_camera")
                rospy.sleep(1.5)  # Wait for camera to detect AR marker
                grab_ok = self.visual_grab(ar_id, buffer_pos)
                if grab_ok:
                    break
                print(f"[重试] 抓取未成功，准备重试...")
            
            if not grab_ok:
                print(f"[警告] 抓取 AR-{ar_id} 最终失败，继续执行下一步")
            
            self.move_arm("safe")
            print("[完成]")
        
        elif action == 'nav_only':
            print(f"\n[执行] 导航到 {nav}")
            self.navigate_to(nav)
            print("[完成]")
        
        elif action == 'charge':
            print(f"\n[执行] 去充电桩")
            self.navigate_to("charging_station")
            print("[完成]")
        
        else:
            print(f"[错误] 未知动作: {action}")
    
    def run_interactive(self):
        print("输入指令测试LLM规划，输入 'q' 退出，输入 'h' 查看帮助\n")
        
        while True:
            try:
                user_input = input(">>> ").strip()
            except EOFError:
                break
            
            if not user_input:
                continue
            
            if user_input.lower() == 'q':
                print("退出")
                break
            
            if user_input.lower() == 'h':
                self.print_help()
                continue
            
            self.process_command(user_input)
    
    def print_help(self):
        print("""
示例指令:
  - "去1号工位"
  - "运3号物块到5号工位"
  - "把4号物块从1号运到3号"
  - "从3号工位取5号物块送到2号工位"
  - "回起始点"

输入 'q' 退出
""")
    
    def process_command(self, user_cmd):
        print(f"\n{'='*60}")
        print(f"  收到指令: {user_cmd}")
        print(f"{'='*60}")
        
        llm_response = self.call_llm(user_cmd)
        if not llm_response:
            print("[错误] LLM响应失败")
            return
        
        print(f"[LLM响应] {llm_response}")
        
        tasks = self.parse_response(llm_response)
        if not tasks:
            print("[错误] 无法解析LLM响应")
            return
        
        if not isinstance(tasks, list):
            tasks = [tasks]
        
        print(f"\n[解析] 共 {len(tasks)} 个任务")
        
        for i, task in enumerate(tasks, 1):
            print(f"\n--- 任务 {i}/{len(tasks)} ---")
            self.execute_task(task)
            # Brief pause between tasks to let Gazebo physics settle
            if i < len(tasks):
                rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        MissionLLM()
    except rospy.ROSInterruptException:
        pass
