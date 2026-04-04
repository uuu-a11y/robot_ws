# Robot Workspace

机械臂+导航综合控制系统

## 系统要求

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+

## 安装依赖

### 1. ROS依赖

```bash
# 安装ROS包
sudo apt update
sudo apt install -y \
    ros-noetic-slam-gmapping \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-joy \
    ros-noetic-map-server
```

### 2. Python依赖

```bash
# Python包通常已随ROS安装
# 如有需要
pip3 install rospkg
```

### 3. 编译工作空间

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 文件结构

```
robot_ws/
├── src/
│   ├── arm_controller/          # 机械臂控制
│   ├── ar_pose/                 # AR识别
│   ├── ar_track_alvar/          # AR追踪
│   ├── relative_move/           # 相对移动控制
│   ├── oryxbot_description/     # 机器人描述
│   ├── oryxbot_navigation/      # 导航包
│   └── oryxbot_slam/            # SLAM建图包
├── maps/                         # 地图文件
├── start_final_mission.sh       # 最终任务启动脚本
├── start_arm_test.sh           # 机械臂测试脚本
├── start_nav_test.sh           # 导航测试脚本
├── start_nav_slam.sh           # SLAM导航脚本
└── COORDINATE_GUIDE.md          # 坐标配置指南
```

## 使用说明

### 1. 最终任务脚本

执行完整的抓取任务：小车导航 → AR对准 → 机械臂抓取 → 放置

```bash
cd ~/robot_ws
./start_final_mission.sh
```

**任务流程**：
1. 小车SLAM导航到5号工位 → AR对准
2. 机械臂抓取AR-3物块 → 放置到台面
3. 小车SLAM导航到1号工位 → AR对准
4. 机械臂抓取AR-9物块 → 放置到台面
5. 小车导航返回起点

### 2. 机械臂测试脚本

测试机械臂抓取功能

```bash
./start_arm_test.sh
```

可选功能：
- 键盘控制
- 菜单控制
- 视觉抓取

### 3. 导航测试脚本

测试相对移动导航

```bash
./start_nav_test.sh
```

### 4. SLAM导航脚本

使用SLAM地图进行导航

```bash
./start_nav_slam.sh
```

可选功能：
- 使用已建地图
- 实时SLAM建图

## 坐标配置

详见 [COORDINATE_GUIDE.md](./COORDINATE_GUIDE.md)

### 机械臂坐标 (mm)

| 位置 | X | Y | Z |
|-----|---|---|---|
| 安全/复位 | 150 | 0 | 200 |
| 摄像头位置 | 90 | 120 | 100 |
| 台面放置 | 80 | -190 | 30 |
| 前储物槽 | 110 | 120 | 40 |
| 后储物槽 | 110 | 180 | 40 |

### 导航坐标 (m)

| 位置 | X | Y |
|-----|---|---|
| 起点 | 0.0 | 0.0 |
| 5号工位 | 0.60 | 1.20 |
| 1号工位 | 2.00 | 2.20 |

## SLAM建图

如需重新建图：

```bash
./start_nav_slam.sh
# 选择 2 启动SLAM建图

# 用键盘控制机器人在环境中移动
# 建好后保存地图
rosrun map_server map_saver -f ~/robot_ws/src/oryxbot_navigation/maps/my_map
```

## 调试工具

### 查看话题
```bash
rostopic list
rostopic echo /topic_name
```

### 查看TF树
```bash
rosrun rqt_tf_tree rqt_tf_tree
```

### 查看节点
```bash
rosnode list
rosnode info /node_name
```

### RViz可视化
```bash
rviz -d ~/robot_ws/src/oryxbot_navigation/rviz/navigation.rviz
```

## 故障排除

### 1. move_base无法启动
确保安装了teb_local_planner：
```bash
sudo apt install ros-noetic-teb-local-planner
```

### 2. 地图加载失败
检查地图文件是否存在：
```bash
ls ~/robot_ws/src/oryxbot_navigation/maps/
```

### 3. AMCL定位失败
确保在RViz中手动设置初始位置 (2D Pose Estimate)

## 许可证

MIT License
