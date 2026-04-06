#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import re
import requests
import time
from std_msgs.msg import String
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from arm_controller.srv import PickPlace, move
from ar_pose.srv import Track

ZHIPU_API_KEY = "50713e5610684389ab3c0ebeacc88ce8.0ntImscaIx3NwTee"
MODEL_TYPE = "glm-4-flash"

NAVI_POINTS = {
    "start": {"x": 0.0, "y": 0.0, "ar_id": 0},
    "station_1": {"x": 2.0, "y": 2.2, "ar_id": 1},
    "station_2": {"x": 2.2, "y": 1.2, "ar_id": 1},
    "station_3": {"x": 2.2, "y": 0.2, "ar_id": 1},
    "station_4": {"x": 0.6, "y": 2.2, "ar_id": 1},
    "station_5": {"x": 0.6, "y": 1.2, "ar_id": 1},
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
        
        rospy.wait_for_service('/move_base/make_plan', timeout=15)
        rospy.wait_for_service('/ar_track', timeout=15)
        rospy.wait_for_service('/goto_position', timeout=15)
        rospy.wait_for_service('/swiftpro/on', timeout=15)
        rospy.wait_for_service('/swiftpro/off', timeout=15)
        
        self.srv_ar_track = rospy.ServiceProxy('/ar_track', Track)
        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)
        
        rospy.sleep(1)
        print(">>> 连接成功！\n")
        
        self.sub_cmd = rospy.Subscriber('/llm_command', String, self.on_command, queue_size=10)
        
        rospy.loginfo("LLM任务规划节点已启动，等待指令...")
        rospy.spin()
    
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
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
        
        self.pub_goal.publish(goal)
        print(f"[导航] 等待 10 秒...")
        rospy.sleep(10)
        return True
    
    def ar_align(self, ar_id):
        if ar_id is None or ar_id <= 0:
            print(f"[AR] 跳过对准（ID={ar_id}）")
            return True
        
        print(f"[AR] 对准 AR-{ar_id}")
        try:
            response = self.srv_ar_track(ar_id=int(ar_id), goal_dist=0.25)
            if response.success:
                print(f"[AR] 对准成功")
            else:
                print(f"[AR] 对准失败: {response.message}")
                return False
        except Exception as e:
            print(f"[AR] 错误: {e}")
            return False
        return True
    
    def move_arm(self, position):
        if position not in ARM_POSITIONS:
            rospy.logerr(f"未知机械臂位置: {position}")
            return False
        
        pos = ARM_POSITIONS[position]
        print(f"[机械臂] 移动到 {position} ({pos['x']}, {pos['y']}, {pos['z']})")
        try:
            self.srv_arm_move(x=pos['x'], y=pos['y'], z=pos['z'])
            rospy.sleep(2)
            return True
        except Exception as e:
            print(f"[机械臂] 错误: {e}")
            return False
    
    def pump_on(self):
        print("[气泵] 开启")
        try:
            self.srv_pump_on()
            rospy.sleep(0.5)
        except:
            pass
    
    def pump_off(self):
        print("[气泵] 关闭")
        try:
            self.srv_pump_off()
            rospy.sleep(0.5)
        except:
            pass
    
    def execute_task(self, task):
        action = task.get('action', '')
        nav = task.get('nav', '')
        ar_id = task.get('ar_id', 0)
        buffer_pos = task.get('buffer', 'buffer_front')
        
        align_id = NAVI_POINTS.get(nav, {}).get('ar_id', 1) if nav else 1
        
        if action == 'car_to_table':
            print(f"\n[执行] 车→台面: AR-{ar_id} 到 {nav}")
            
            if nav:
                self.navigate_to(nav)
                self.ar_align(align_id)
            
            self.move_arm("camera")
            self.ar_align(ar_id)
            self.pump_on()
            
            self.move_arm("table_place")
            self.pump_off()
            
            self.move_arm("safe")
            print("[完成]")
        
        elif action == 'table_to_car':
            print(f"\n[执行] 台面→车: {nav} AR-{ar_id}")
            
            if nav:
                self.navigate_to(nav)
                self.ar_align(align_id)
            
            self.move_arm("table_camera")
            self.ar_align(ar_id)
            self.pump_on()
            
            self.move_arm(buffer_pos)
            self.pump_off()
            
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
    
    def on_command(self, msg):
        user_cmd = msg.data.strip()
        if not user_cmd:
            return
        
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

if __name__ == '__main__':
    try:
        MissionLLM()
    except rospy.ROSInterruptException:
        pass
