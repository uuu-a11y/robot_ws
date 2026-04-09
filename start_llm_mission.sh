#!/bin/bash

cleanup() {
    echo "正在关闭所有进程..."
    kill $(jobs -p) 2>/dev/null
    killall -9 gzserver gzclient rosmaster rosout nodelet 2>/dev/null
    echo ">>> 环境已清理完毕。"
}

trap cleanup EXIT

# 轮询等待ROS话题出现
wait_for_topic() {
    local topic=$1 timeout=$2
    local elapsed=0
    echo "  - 等待话题 $topic (最长${timeout}s)..."
    while ! rostopic list 2>/dev/null | grep -q "$topic" && [ $elapsed -lt $timeout ]; do
        sleep 1
        ((elapsed++))
    done
    if [ $elapsed -lt $timeout ]; then
        echo "  - 话题 $topic 已就绪 (${elapsed}s)"
    else
        echo "  - 话题 $topic 等待超时，继续执行"
    fi
}

# 轮询等待ROS服务出现
wait_for_service() {
    local srv=$1 timeout=$2
    local elapsed=0
    echo "  - 等待服务 $srv (最长${timeout}s)..."
    while ! rosservice list 2>/dev/null | grep -q "$srv" && [ $elapsed -lt $timeout ]; do
        sleep 1
        ((elapsed++))
    done
    if [ $elapsed -lt $timeout ]; then
        echo "  - 服务 $srv 已就绪 (${elapsed}s)"
    else
        echo "  - 服务 $srv 等待超时，继续执行"
    fi
}

echo "=========================================="
echo "   LLM任务规划启动脚本"
echo "=========================================="

echo "[1/7] 清理旧环境..."
killall -9 gzserver gzclient rosmaster 2>/dev/null
sleep 2

echo "[2/7] 加载环境变量..."
source ~/robot_ws/devel/setup.bash

# 抑制 TF_REPEATED_DATA 警告（仿真时钟下 ar_track_alvar 的已知问题）
export ROSCONSOLE_CONFIG_FILE=~/robot_ws/src/oryxbot_description/config/rosconsole.config
export ROSCONSOLE_MIN_SEVERITY=2  # 只显示 ERROR 和 FATAL

echo "[3/7] 启动仿真环境..."
roslaunch oryxbot_description gazebo.launch > /dev/null 2>&1 &
wait_for_topic /clock 30

echo "[4/7] 启动导航与AR功能..."
roslaunch oryxbot_description ar_base_gazebo.launch > /dev/null 2>&1 &
wait_for_service /ar_track 15

echo "[5/7] 启动机械臂抓取功能..."
roslaunch oryxbot_description pick_ar_gazebo.launch > /dev/null 2>&1 &
sleep 2

echo "启动IK节点..."
rosrun oryxbot_description ik_swiftpro &
wait_for_service /goto_position 10

echo "[6/7] 启动SLAM导航..."
echo "  - 启动 map_server..."
rosrun map_server map_server ~/robot_ws/src/oryxbot_navigation/maps/my_map.yaml > /dev/null 2>&1 &
wait_for_topic /map 10

echo "  - 启动 AMCL 定位..."
roslaunch oryxbot_navigation amcl_sim.launch > /dev/null 2>&1 &
wait_for_topic /amcl_pose 10

echo "  - 启动 move_base..."
roslaunch oryxbot_navigation move_base_sim.launch > /dev/null 2>&1 &
wait_for_topic /move_base/status 15

echo "=========================================="
echo "   仿真环境已就绪"
echo "=========================================="
echo ""
echo "现在输入 LLM 命令控制小车..."
echo "示例: 运3号物块到5号工位"
echo "输入 'q' 退出"
echo "=========================================="
echo ""

echo "[7/7] 启动LLM任务规划..."
cd ~/robot_ws
rosrun oryxbot_description mission_llm.py
