#!/bin/bash

cleanup() {
    echo "正在关闭所有进程..."
    kill $(jobs -p) 2>/dev/null
    killall -9 gzserver gzclient rosmaster rosout nodelet rviz 2>/dev/null
    echo ">>> 环境已清理完毕。"
}

trap cleanup EXIT

echo "=========================================="
echo "   导航调试启动脚本"
echo "=========================================="

echo "[1/4] 清理旧环境..."
killall -9 gzserver gzclient rosmaster rviz 2>/dev/null
sleep 2

echo "[2/4] 加载环境变量..."
source ~/robot_ws/devel/setup.bash

echo "[3/4] 启动导航环境..."

echo "  - 启动 Gazebo..."
roslaunch oryxbot_description gazebo.launch > /dev/null 2>&1 &
sleep 15

echo "  - 启动导航与AR对准节点..."
roslaunch oryxbot_description ar_base_gazebo.launch > /dev/null 2>&1 &
sleep 3

echo "  - 启动 RViz..."
rviz -d ~/robot_ws/src/oryxbot_description/urdf.rviz > /dev/null 2>&1 &
sleep 2

echo ""
echo "=========================================="
echo "   导航环境已就绪!"
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
    python3 nav_test.py
else
    echo "启动键盘控制..."
    cd ~/robot_ws/src/oryxbot_description/src/
    python3 nav_keyboard.py
fi

echo "按 Enter 退出..."
read dummy
