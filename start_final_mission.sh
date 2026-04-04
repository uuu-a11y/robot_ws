#!/bin/bash

cleanup() {
    echo "正在关闭所有进程..."
    kill $(jobs -p) 2>/dev/null
    killall -9 gzserver gzclient rosmaster rosout 2>/dev/null
    echo ">>> 环境已清理完毕。"
}

trap cleanup EXIT

echo "=========================================="
echo "   最终任务启动脚本"
echo "=========================================="

echo "[1/5] 清理旧环境..."
killall -9 gzserver gzclient rosmaster 2>/dev/null
sleep 2

echo "[2/5] 加载环境变量..."
source ~/robot_ws/devel/setup.bash

echo "[3/5] 启动仿真环境..."

echo "  - 启动 Gazebo..."
roslaunch oryxbot_description gazebo.launch > /dev/null 2>&1 &
sleep 15

echo "  - 启动相对移动和AR节点..."
roslaunch oryxbot_description ar_base_gazebo.launch > /dev/null 2>&1 &
sleep 3

echo "  - 启动机械臂IK节点..."
rosrun oryxbot_description ik_swiftpro > /dev/null 2>&1 &
sleep 2

echo "  - 启动手部相机AR识别..."
roslaunch ar_pose ar_pick_sim.launch > /dev/null 2>&1 &
sleep 2

echo "  - 启动视觉抓取节点..."
roslaunch oryxbot_description pick_ar_gazebo.launch > /dev/null 2>&1 &
sleep 2

echo "  - 启动 robot_state_publisher..."
rosrun robot_state_publisher robot_state_publisher > /dev/null 2>&1 &
sleep 1

echo ""
echo "[4/5] 启动SLAM导航..."

echo "  - 启动 map_server..."
rosrun map_server map_server ~/robot_ws/src/oryxbot_navigation/maps/my_map.yaml > /dev/null 2>&1 &
sleep 2

echo "  - 启动 AMCL 定位..."
roslaunch oryxbot_navigation amcl_sim.launch > /dev/null 2>&1 &
sleep 2

echo "  - 启动 move_base..."
roslaunch oryxbot_navigation move_base_sim.launch > /dev/null 2>&1 &
sleep 2

echo ""
echo "=========================================="
echo "   环境已就绪!"
echo "=========================================="
echo ""
echo "提示: 使用外部 RViz 连接到此 ROS master 可以查看状态"
echo ""

echo "按 Enter 启动最终任务..."
read dummy

echo ""
echo "=========================================="
echo "   启动最终任务..."
echo "=========================================="
echo ""

cd ~/robot_ws/src/oryxbot_description/src/
python3 mission_final.py

echo ""
echo "按 Enter 退出..."
read dummy
