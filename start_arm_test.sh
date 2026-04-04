#!/bin/bash

cleanup() {
    echo "正在关闭所有进程..."
    kill $(jobs -p) 2>/dev/null
    killall -9 gzserver gzclient rosmaster rosout nodelet rviz 2>/dev/null
    echo ">>> 环境已清理完毕。"
}

trap cleanup EXIT

echo "=========================================="
echo "   机械臂测试启动脚本"
echo "=========================================="

echo "[1/4] 清理旧环境..."
killall -9 gzserver gzclient rosmaster rviz 2>/dev/null
sleep 2

echo "[2/4] 加载环境变量..."
source ~/robot_ws/devel/setup.bash

echo "[3/4] 启动测试环境..."

echo "  - 启动 Gazebo..."
roslaunch oryxbot_description gazebo.launch > /dev/null 2>&1 &
sleep 15

echo "  - 启动 IK 节点..."
rosrun oryxbot_description ik_swiftpro > /dev/null 2>&1 &
sleep 2

echo "  - 启动手部相机 AR 识别..."
roslaunch ar_pose ar_pick_sim.launch > /dev/null 2>&1 &
sleep 2

echo "  - 启动视觉抓取节点..."
roslaunch oryxbot_description pick_ar_gazebo.launch > /dev/null 2>&1 &
sleep 2

echo "  - 启动 RViz..."
rviz -d ~/robot_ws/src/oryxbot_description/urdf.rviz > /dev/null 2>&1 &
sleep 2

echo ""
echo "=========================================="
echo "   测试环境已就绪!"
echo "=========================================="
echo ""
echo "选择控制模式:"
echo "  1. 键盘控制 (推荐)"
echo "  2. 菜单控制"
echo ""
echo -n "选择 [1]: "
read choice

if [ "$choice" = "2" ]; then
    echo "启动菜单控制..."
    cd ~/robot_ws/src/oryxbot_description/src/
    python3 arm_test.py
else
    echo "启动键盘控制..."
    cd ~/robot_ws/src/oryxbot_description/src/
    python3 arm_keyboard.py
fi

echo "按 Enter 退出..."
read dummy
