#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import re
import requests

ZHIPU_API_KEY = "50713e5610684389ab3c0ebeacc88ce8.0ntImscaIx3NwTee"
MODEL_TYPE = "glm-4-flash"

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

def call_llm(user_message):
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

def parse_response(response_text):
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

def print_plan(task):
    if isinstance(task, list):
        for i, t in enumerate(task, 1):
            print_plan_single(t, i)
        print("=" * 60 + "\n")
    else:
        print_plan_single(task, 1)
        print("=" * 60 + "\n")

def print_plan_single(task, index=1):
    print("\n" + "=" * 60)
    print(f"  任务 {index}")
    print("=" * 60)
    
    action = task.get('action', '')
    nav = task.get('nav', '')
    ar_id = task.get('ar_id', 0)
    buffer_pos = task.get('buffer', 'buffer_front')
    desc = task.get('description', '')
    
    print(f"描述: {desc}")
    print(f"动作: {action}")
    print(f"导航点: {nav}")
    print(f"物块ID: {ar_id}")
    print()
    
    if action == 'car_to_table':
        print("执行步骤:")
        print(f"  1. 导航到 {nav} → AR对准(ID=1)")
        print(f"  2. 移动到 camera（摄像头位置）")
        print(f"  3. AR对准抓取 AR-{ar_id}")
        print(f"  4. 移动到 table_place（台面放置位）")
        print(f"  5. 放置物块")
        print(f"  6. 归位 safe")
    
    elif action == 'table_to_car':
        print("执行步骤:")
        print(f"  1. 导航到 {nav} → AR对准(ID=1)")
        print(f"  2. 移动到 table_camera（台面相机位）")
        print(f"  3. AR对准抓取 AR-{ar_id}")
        print(f"  4. 移动到 {buffer_pos}（储物槽）")
        print(f"  5. 放置物块")
        print(f"  6. 归位 safe")
    
    elif action == 'nav_only':
        print("执行步骤:")
        print(f"  1. 导航到 {nav}")
        print(f"  2. 等待完成")
    
    elif action == 'charge':
        print("执行步骤:")
        print(f"  1. 导航到 charging_station（充电桩）")
        print(f"  2. 面向正左方")
        print(f"  3. 开始充电")

def main():
    print("=" * 60)
    print("  LLM 任务规划测试")
    print("=" * 60)
    print()
    print("输入指令测试LLM规划逻辑，输入 'q' 退出")
    print()
    
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
            print_help()
            continue
        
        print(f"\n[发送] {user_input}")
        
        response = call_llm(user_input)
        if not response:
            print("[错误] 未收到响应\n")
            continue
        
        print(f"[LLM] {response}")
        
        task = parse_response(response)
        if task:
            print_plan(task)
        else:
            print("[提示] 无法解析JSON\n")

def print_help():
    print("""
示例指令:
  - "去1号工位"
  - "运3号物块到5号工位"
  - "把5号物块放到车上"
  - "去充电桩"
  - "把小车上的物块从station_1送到station_3"

输入 'q' 退出
""")

if __name__ == '__main__':
    main()
