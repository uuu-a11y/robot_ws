# Robot Workspace

机械臂 + 自主导航综合控制系统（ROS Noetic）

## 系统要求

- Ubuntu 20.04
- ROS Noetic
- Python 3.8+

## 快速开始

### 1. 安装 ROS 依赖

```bash
sudo apt update
sudo apt install -y \
    ros-noetic-slam-gmapping \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-joy \
    ros-noetic-map-server \
    ros-noetic-moveit \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    ros-noetic-driver-base \
    ros-noetic-ackermann-msgs

pip3 install rospkg
```

### 2. 下载 ar_track_alvar 源码

```bash
cd ~/robot_ws/src
git clone https://gitee.com/reinovo/ar_track_alvar.git
```

### 3. 配置 Gazebo 插件

本项目使用 contact_plugin 进行物块碰撞检测：

```bash
# 下载插件源码（如还没有）
git clone https://gitee.com/xk-fly/moliyuanbao.git /tmp/moliyuanbao
cp -r /tmp/moliyuanbao/gazebo_plugins ~/

# 配置环境变量（Ubuntu 20.04）
echo 'export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo_plugins/contact_plugin_20-04_g11' >> ~/.bashrc
source ~/.bashrc
```

### 4. 配置 Gazebo 模型

将场地模型复制到 Gazebo 模型目录：

```bash
cp -r /tmp/moliyuanbao/models/* ~/.gazebo/models/
```

模型包括：AR 轨道码（id1-id9）、围栏、加工台、充电桩、地图等。

### 5. 编译

```bash
cd ~/robot_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

# 永久生效（可选）
echo "source ~/robot_ws/devel/setup.bash" >> ~/.bashrc
```

## 启动脚本

| 脚本 | 用途 |
|------|------|
| `./start_robot.sh` | **综合调试环境**（推荐日常使用） |
| `./start_llm_mission.sh` | LLM 智能任务规划 |
| `./start_final_mission.sh` | 固定路线任务（5号→1号工位） |
| `./start_arm_test.sh` | 机械臂单独测试 |
| `./start_nav_slam.sh` | SLAM 建图 + 导航测试 |

### 1. 综合调试环境（推荐）

导航 + 机械臂一体化键盘控制，Tab 切换模式：

```bash
./start_robot.sh
```

快捷键：`H` 查看帮助，`Tab` 切换导航/机械臂，数字键 `1-6` 快速导航或机械臂预设位。

### 2. LLM 任务规划

通过自然语言控制机器人执行任务：

```bash
./start_llm_mission.sh
```

使用前需配置 API Key（见 [COORDINATE_GUIDE.md](./COORDINATE_GUIDE.md#六llm任务规划)）。

支持的指令示例：
- "去1号工位" → 仅导航
- "运3号物块到5号工位" → 抓取 + 运输
- "去充电桩" → 导航到充电站

### 3. 固定路线任务

自动执行：5号工位抓AR-3 → 1号工位抓AR-9 → 返回起点

```bash
./start_final_mission.sh
```

### 4. 机械臂测试

```bash
./start_arm_test.sh
```

### 5. SLAM 建图 / 导航

```bash
./start_nav_slam.sh
# 选1: 使用已建地图导航（推荐）
# 选2: 实时 SLAM 建图
```

建好图后保存：
```bash
rosrun map_server map_saver -f ~/robot_ws/src/oryxbot_navigation/maps/my_map
```

## 项目结构

```
robot_ws/
├── src/
│   ├── oryxbot_description/    # 机器人模型 + 机械臂控制 + 任务脚本
│   │   ├── src/
│   │   │   ├── robot_control.py     # 综合控制台（Tab切换导航/机械臂）
│   │   │   ├── mission_llm.py       # LLM 任务规划节点
│   │   │   ├── mission_final.py     # 固定路线任务脚本
│   │   │   ├── mission_controller.py # 旧版任务脚本（相对移动）
│   │   │   ├── ik_swiftpro.cpp      # 逆运动学节点
│   │   │   ├── pick_ar_gazebo.cpp   # 视觉抓取节点
│   │   │   └── coordinates.json     # 导航/机械臂坐标配置
│   │   ├── launch/                  # Gazebo仿真 + AR + 抓取 launch文件
│   │   ├── config/                  # 控制器 + RViz 配置
│   │   └── world/                   # Gazebo 世界文件
│   ├── oryxbot_navigation/      # 导航包
│   │   ├── param/                    # TEB/Costmap 参数
│   │   ├── maps/                     # SLAM 地图文件
│   │   └── launch/                   # AMCL + move_base launch文件
│   ├── oryxbot_slam/            # SLAM 建图包
│   ├── ar_pose/                 # AR 码识别 + 底盘对准
│   ├── ar_track_alvar/          # AR 追踪库
│   ├── arm_controller/          # 机械臂服务接口
│   ├── relative_move/           # 相对移动（避障）
│   └── pid_lib/                 # PID 控制库
├── start_robot.sh               # 综合调试（推荐）
├── start_llm_mission.sh         # LLM 任务
├── start_final_mission.sh       # 固定路线任务
├── start_arm_test.sh            # 机械臂测试
├── start_nav_slam.sh            # SLAM/导航
├── COORDINATE_GUIDE.md          # 坐标配置指南
└── README.md
```

## 坐标配置

详见 [COORDINATE_GUIDE.md](./COORDINATE_GUIDE.md)

## 自定义 AR 物料

如需添加新的 AR 码物块，在 `~/.gazebo/models/` 下创建模型文件夹：

```
marker_id3/
├── materials/
│   ├── scripts/reinovo.material   # 材质定义（关联图片纹理）
│   └── textures/id3.png           # AR 码图片
├── model.config                   # 模型描述
└── model.sdf                      # 物理属性、视觉、关节
```

参考现有模型 `~/.gazebo/models/ar_track_3/` 进行修改。

## 任务流程

本项目对应**睿抗机器人大赛 - 魔力元宝**赛项：

1. **任务一**：小车导航到5号工位 → AR对准 → 从车上 Buffer 抓取 ID-3 物块 → 放置到台面
2. **任务二**：小车导航到1号工位 → AR对准 → 视觉识别并抓取 ID-9 物块 → 放置到车上
3. **返回起点**

场地布局：起点(0,0)、5号工位(0.6,1.2)、1号工位(2.0,2.2)、充电桩(0.4,2.0)

> 详细坐标参数见 [COORDINATE_GUIDE.md](./COORDINATE_GUIDE.md)

## 调试工具

```bash
rostopic list                    # 查看话题
rostopic echo /topic_name        # 监听话题
rosnode list                     # 查看节点
rosrun rqt_tf_tree rqt_tf_tree   # 查看 TF 树
```

## 故障排除

| 问题 | 解决方案 |
|------|----------|
| move_base 无法启动 | `sudo apt install ros-noetic-teb-local-planner` |
| 地图加载失败 | 检查 `src/oryxbot_navigation/maps/my_map.yaml` 是否存在 |
| AMCL 定位失败 | 在 RViz 中用 "2D Pose Estimate" 设置初始位置 |
| TF 警告刷屏 | 已在启动脚本中自动抑制（ROSCONSOLE_CONFIG_FILE） |
| Gazebo 启动失败/模型缺失 | 检查 `~/.gazebo/models/` 是否有完整模型文件 |
| 物块碰撞不生效 | 确认 `GAZEBO_PLUGIN_PATH` 包含 contact_plugin 路径 |
| AR 码识别不到 | 检查摄像头话题 `rostopic echo /ar_pose_marker`，确认光照和距离 |

## 许可证

MIT License
