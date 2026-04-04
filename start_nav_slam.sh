#!/bin/bash

cleanup() {
    echo "正在关闭所有进程..."
    kill $(jobs -p) 2>/dev/null
    killall -9 gzserver gzclient rosmaster rosout rviz 2>/dev/null
    echo ">>> 环境已清理完毕。"
}

trap cleanup EXIT

echo "=========================================="
echo "   SLAM 导航仿真启动脚本"
echo "=========================================="

echo "[1/5] 清理旧环境..."
killall -9 gzserver gzclient rosmaster rviz 2>/dev/null
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

echo ""
echo "=========================================="
echo "   选择地图模式"
echo "=========================================="
echo ""
echo "  1. 使用已建地图 (my_map.pgm) - 推荐"
echo "  2. 启动 SLAM 实时建图"
echo "  0. 不启动地图"
echo ""
echo -n "选择 [1]: "
read map_choice

if [ "$map_choice" = "2" ]; then
    echo "  启动 SLAM 建图..."
    roslaunch oryxbot_slam oryxbot_gmapping_sim.launch > /dev/null 2>&1 &
    sleep 3
    echo "  启动 RViz (SLAM)..."
    rviz -d ~/robot_ws/src/oryxbot_slam/rviz/oryxbot_gmapping_sim.rviz > /dev/null 2>&1 &
elif [ "$map_choice" != "0" ]; then
    echo "  加载地图 (my_map.pgm)..."
    rosrun map_server map_server ~/robot_ws/src/oryxbot_navigation/maps/my_map.yaml > /dev/null 2>&1 &
    sleep 2
    echo "  启动 AMCL 定位..."
    roslaunch oryxbot_navigation amcl_sim.launch > /dev/null 2>&1 &
    sleep 2
    echo "  启动 move_base..."
    roslaunch oryxbot_navigation move_base_sim.launch > /dev/null 2>&1 &
    sleep 2
    echo "  启动 RViz (导航)..."
    rviz -d ~/robot_ws/src/oryxbot_navigation/rviz/navigation.rviz > /dev/null 2>&1 &
    sleep 3
else
    echo "  不加载地图"
    echo "  启动 RViz..."
    rviz -d ~/robot_ws/src/oryxbot_description/urdf.rviz > /dev/null 2>&1 &
fi

sleep 2

echo ""
echo "=========================================="
echo "   环境已就绪!"
echo "=========================================="
echo ""
echo "请先在 RViz 中:"
echo "  1. 点击 '2D Pose Estimate' 设置机器人初始位置 (绿色箭头)"
echo "  2. 等待粒子云收敛"
echo ""
echo "选择控制模式:"
echo "  1. 键盘控制 (机械臂)"
echo "  2. 菜单控制 (机械臂)"
echo "  3. 键盘控制 (导航)"
echo "  4. 菜单控制 (导航)"
echo "  5. 发送测试导航目标 (0.5, 0.5)"
echo "  0. 退出"
echo -n "选择: "
read choice

case $choice in
    1)
        cd ~/robot_ws/src/oryxbot_description/src/
        python3 arm_keyboard.py
        ;;
    2)
        cd ~/robot_ws/src/oryxbot_description/src/
        python3 arm_test.py
        ;;
    3)
        cd ~/robot_ws/src/oryxbot_description/src/
        python3 nav_keyboard.py
        ;;
    4)
        cd ~/robot_ws/src/oryxbot_description/src/
        python3 nav_test.py
        ;;
    5)
        echo "发送测试导航目标到 (0.5, 0.5)..."
        python3 ~/robot_ws/src/oryxbot_description/src/nav_goal_test.py 0.5 0.5 0
        ;;
    *)
        echo "退出..."
        ;;
esac

echo "按 Enter 退出..."
read dummy
