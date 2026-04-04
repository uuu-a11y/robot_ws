#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Empty
from arm_controller.srv import PickPlace, move
from arm_controller.msg import control
from relative_move.srv import SetRelativeMove
from ar_pose.srv import Track

# ================= 🗺️ 核心坐标与参数配置 =================

# --- 1. 场地关键点 (根据你的标定数据) ---
START_X = 0.001
START_Y = -0.001

# 5号加工中心 (任务一：上料点)
STATION_5_X = 0.755
STATION_5_Y = 1.162
STATION_5_ID = 5  

# 1号加工中心 (任务二：取料点)
STATION_1_X = 2.221
STATION_1_Y = 2.144
STATION_1_ID = 1 

# --- 2. 机械臂动作参数 (单位: mm) ---
# Buffer (车上的储物槽)
# 设定：左侧放ID-3，右侧放ID-9 (为了防止打架)
BUFFER_LEFT_X = 0.0     # 前后
BUFFER_LEFT_Y = 180.0   # 左右 (左侧)
BUFFER_RIGHT_X = 0.0
BUFFER_RIGHT_Y = -180.0 # 右侧 (如果buff够宽) 或者就用同一个位置分先后

BUFFER_Z_LIFT = 100.0   # 抬起高度
BUFFER_Z_GRAB = 45.0    # 抓取高度

# 加工台交互位置
TABLE_EXTEND_X = 300.0  # 伸出距离
TABLE_EXTEND_Y = 0.0    # 正前方
TABLE_Z_DROP = 200.0    # 放置高度
TABLE_Z_HOVER = 240.0   # 悬停高度

# ===============================================================

class MissionCommander:
    def __init__(self):
        rospy.init_node('mission_commander', anonymous=True)
        
        rospy.loginfo(">>> 正在连接机器人底层服务...")
        try:
            rospy.wait_for_service('/relative_move', timeout=10)
            rospy.wait_for_service('/ar_track', timeout=10)
            rospy.wait_for_service('/pick_ar', timeout=10)
            rospy.wait_for_service('/goto_position', timeout=10)
            rospy.wait_for_service('/swiftpro/on', timeout=10)
        except rospy.ROSException:
            rospy.logerr("连接超时！请检查 run_mission.sh 是否正确启动了所有节点。")
            exit(1)
        
        self.srv_move_base = rospy.ServiceProxy('/relative_move', SetRelativeMove)
        self.srv_align = rospy.ServiceProxy('/ar_track', Track)
        self.srv_pick_ar = rospy.ServiceProxy('/pick_ar', PickPlace)
        self.srv_arm_move = rospy.ServiceProxy('/goto_position', move)
        self.srv_pump_on = rospy.ServiceProxy('/swiftpro/on', Empty)
        self.srv_pump_off = rospy.ServiceProxy('/swiftpro/off', Empty)
        
        rospy.loginfo(">>> 系统就绪。")

    def move_chassis(self, target_x, target_y):
        """底盘移动 (带避障)"""
        dx = target_x - START_X
        dy = target_y - START_Y
        rospy.loginfo(f"🚗 导航启动 -> 目标:({target_x:.2f}, {target_y:.2f})")
        
        try:
            goal_pose = Pose2D()
            goal_pose.x = dx
            goal_pose.y = dy
            goal_pose.theta = 0.0 
            
            # 开启避障: avoid_obstacle=True
            self.srv_move_base(goal_pose, "odom", True, False)
            time.sleep(1) 
        except Exception as e:
            rospy.logerr(f"底盘移动失败: {e}")

    def align_tag(self, tag_id):
        """视觉精准泊车"""
        rospy.loginfo(f"👀 视觉对准 -> ID-{tag_id}")
        try:
            self.srv_align(ar_id=tag_id, goal_dist=0.3)
            time.sleep(1)
        except Exception as e:
            rospy.logwarn(f"对准失败 (可能已被挡住): {e}")

    def move_arm(self, x, y, z):
        """机械臂简单移动"""
        p = control()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.roll = 0.0
        p.pitch = 0.0
        p.yaw = 0.0
        self.srv_arm_move(pose=p)
        time.sleep(0.8)

    # ================= 任务流程 =================

    def run_task_1(self):
        """任务一：将 Buffer 里的 ID-3 送到 5号台"""
        rospy.loginfo("\n========== [任务一] 上料 (ID-3 -> 5号台) ==========")
        
        # 1. 导航
        self.move_chassis(STATION_5_X, STATION_5_Y)
        self.align_tag(STATION_5_ID)
        
        # 2. 机械臂动作 (盲抓 Buffer -> 放台子)
        rospy.loginfo("   >>> 动作：从车上抓取 ID-3 放到台面")
        self.move_arm(150, 0, 150) # 复位
        
        # 抓取 Buffer 左侧
        self.move_arm(BUFFER_LEFT_X, BUFFER_LEFT_Y, BUFFER_Z_LIFT)
        self.move_arm(BUFFER_LEFT_X, BUFFER_LEFT_Y, BUFFER_Z_GRAB)
        self.srv_pump_on()
        time.sleep(0.5)
        self.move_arm(BUFFER_LEFT_X, BUFFER_LEFT_Y, BUFFER_Z_LIFT)
        
        # 放置到台面
        self.move_arm(TABLE_EXTEND_X, TABLE_EXTEND_Y, TABLE_Z_HOVER)
        self.move_arm(TABLE_EXTEND_X, TABLE_EXTEND_Y, TABLE_Z_DROP)
        self.srv_pump_off()
        time.sleep(0.5)
        self.move_arm(150, 0, 150) # 收回
        
        # 3. 返回起点
        rospy.loginfo("   >>> 返回起点...")
        self.move_chassis(START_X, START_Y)

    def run_task_2_setup(self):
        """任务二(准备阶段)：把 ID-9 送到 1号台 (帮你布置考场)"""
        rospy.loginfo("\n========== [任务二·准备] 布置 ID-9 到 1号台 ==========")
        rospy.loginfo("提示：请确保你已经在 Gazebo 里把 ID-9 方块放到了小车Buffer上！")
        
        # 1. 导航
        self.move_chassis(STATION_1_X, STATION_1_Y)
        self.align_tag(STATION_1_ID)
        
        # 2. 放置动作 (盲抓 Buffer -> 放台子)
        # 假设 ID-9 也放在 Buffer 左侧 (因为任务一已经把那里的 ID-3 拿走了，空出来了)
        self.move_arm(150, 0, 150)
        self.move_arm(BUFFER_LEFT_X, BUFFER_LEFT_Y, BUFFER_Z_LIFT)
        self.move_arm(BUFFER_LEFT_X, BUFFER_LEFT_Y, BUFFER_Z_GRAB)
        self.srv_pump_on()
        time.sleep(0.5)
        self.move_arm(BUFFER_LEFT_X, BUFFER_LEFT_Y, BUFFER_Z_LIFT)
        
        self.move_arm(TABLE_EXTEND_X, TABLE_EXTEND_Y, TABLE_Z_HOVER)
        self.move_arm(TABLE_EXTEND_X, TABLE_EXTEND_Y, TABLE_Z_DROP)
        self.srv_pump_off()
        time.sleep(0.5)
        self.move_arm(150, 0, 150)
        
        # 3. 返回起点 (准备开始正式考试)
        rospy.loginfo("   >>> 布置完成，返回起点，准备正式取料...")
        self.move_chassis(START_X, START_Y)
        time.sleep(2)

    def run_task_2_execute(self):
        """任务二(正式阶段)：把 ID-9 从 1号台取回来"""
        rospy.loginfo("\n========== [任务二·正式] 从 1号台取回 ID-9 ==========")
        
        # 1. 导航
        self.move_chassis(STATION_1_X, STATION_1_Y)
        self.align_tag(STATION_1_ID)
        
        # 2. 智能抓取 (使用 pick_ar 视觉识别)
        rospy.loginfo("   >>> 动作：视觉寻找 ID-9 并抓回")
        
        buffer_target = control()
        buffer_target.position.x = BUFFER_LEFT_X
        buffer_target.position.y = BUFFER_LEFT_Y
        buffer_target.position.z = BUFFER_Z_GRAB + 20
        buffer_target.roll = 0.0
        buffer_target.pitch = 0.0
        buffer_target.yaw = 0.0
        
        # 调用视觉抓取服务
        self.srv_pick_ar(number=9, mode=0, pose=buffer_target)
        
        # 3. 返回起点
        time.sleep(2)
        rospy.loginfo("   >>> 取料完成，返回起点...")
        self.move_chassis(START_X, START_Y)

    def run(self):
        # 1. 执行任务一
        self.run_task_1()
        
        time.sleep(3)
        
        # 2. 执行任务二 (按照你的思路：先送过去，再取回来)
        self.run_task_2_setup()   # 阶段A：送过去
        self.run_task_2_execute() # 阶段B：取回来
        
        rospy.loginfo("\n🏆 所有任务圆满结束！")

if __name__ == "__main__":
    cmdr = MissionCommander()
    # ⚠️ 重要提示：
    # 为了让代码能抓到东西，运行脚本前，请在 Gazebo 里
    # 手动把 ID-3 和 ID-9 两个方块，拖拽放到小车后斗上！
    cmdr.run()