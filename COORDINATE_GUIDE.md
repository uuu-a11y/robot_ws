# 坐标配置指南

## 一、启动脚本

```bash
cd ~/robot_ws
source devel/setup.bash

./start_arm_test.sh        # 机械臂调试（键盘/菜单控制）
./start_nav_test.sh        # 导航测试
./start_nav_slam.sh        # SLAM导航（选1用地图，选2实时建图）
./start_final_mission.sh   # 完整任务
./start_robot.sh           # 机器人完整启动

rosrun oryxbot_description debug_final.py  # 统一调试脚本（推荐）
rosrun oryxbot_description mission_llm.py   # LLM任务规划
python3 src/oryxbot_description/src/llm_test.py  # LLM规划测试（不需要ROS）
```

---

## 二、机械臂坐标（单位：mm）

| 名称 | X | Y | Z | 说明 |
|------|---|---|---|------|
| safe | 150 | 0 | 100 | 安全归位 |
| camera | 90 | 120 | 100 | 摄像头位置（面向储物槽） |
| table_place | 60 | -230 | 30 | 台面放置 |
| buffer_front | 110 | 120 | 40 | 前储物槽 |
| buffer_back | 110 | 180 | 40 | 后储物槽 |
| table_camera | 110 | -170 | 100 | 台面相机位（面向台面） |

**坐标范围：** X: 0-280 | Y: -278~278 | Z: 0-130

---

## 三、导航坐标（单位：m）

| 名称 | X | Y | AR | 朝向 | 说明 |
|------|---|---|---|------|------|
| start | 0.0 | 0.0 | 0 | - | 起点 |
| station_1 | 2.0 | 2.2 | 1 | - | 1号工位 |
| station_2 | 2.2 | 1.2 | 1 | - | 2号工位 |
| station_3 | 2.2 | 0.2 | 1 | - | 3号工位 |
| station_4 | 0.6 | 2.2 | 1 | - | 4号工位 |
| station_5 | 0.6 | 1.2 | 1 | - | 5号工位 |
| charging_station | 0.4 | 2.0 | 0 | 90° | 充电桩（朝向正左） |

---

## 四、统一调试脚本快捷键

| 按键 | 功能 |
|------|------|
| W/A/S/D | X/Y轴移动 |
| Q/E | Z轴上下 |
| 方向键 | 微调 |
| 空格 | 归零/气泵 |
| C | 命令菜单 |

---

## 五、重要参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 对准距离 | 0.25m | AR对准目标距离 |
| 导航等待 | 10秒 | 导航后等待时间 |
| AR目标 | 3, 9 | 抓取物块ID |

---

## 六、LLM任务规划

### 配置API Key
```bash
# 编辑 mission_llm.py，修改以下两行：
MINIMAX_API_KEY = "your_api_key"  # 替换为你的API Key
MINIMAX_API_URL = "your_api_url"   # 替换为你的API地址
```

### 使用方法
```bash
# 终端1：启动机器人
./start_robot.sh

# 终端2：发布指令
rostopic pub /llm_command std_msgs/String "把物块从5号工位送到3号工位"
```

### 支持的指令类型

| 指令示例 | 动作 | 说明 |
|---------|------|------|
| "去1号工位" | nav_only | 仅导航 |
| "运3号物块到5号工位" | car_to_table | 从车上运到台面 |
| "把5号物块放到车上" | table_to_car | 从台面运到车上 |
| "去充电桩" | charge | 去充电 |

### 注意事项
- 小车→台面：camera → AR抓取 → table_place → 放置
- 台面→小车：table_camera → AR抓取 → buffer → 放置
- station_1~5 的 AR对准ID=1，charging_station 的 AR对准ID=0

---

*更新：2026-04-06*
