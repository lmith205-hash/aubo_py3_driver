#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import math
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# ROS KDL 库 (基于 Python 3)
import PyKDL
from kdl_parser_py.urdf import treeFromFile

# 引入通用驱动
from unified_driver import AuboDriver

# ================= 配置区域 =================
USE_SIMULATION = True  # True: 仿真; False: 真机
REAL_IP = "100.100.1.10"
# URDF 必须准确，KDL 依赖它
URDF_PATH = "/home/lmy/aubo_i12/src/aubo_robot/aubo_robot/aubo_description/urdf/aubo_i12.urdf"

# KDL 链条配置 (请核对 URDF 中的 Link 名称)
BASE_LINK = "base_link"
TIP_LINK = "wrist3_Link"

# --- 导纳参数 (手感调整) ---
# [修改] 质量 M: 3.0 -> 1.0
# 减小质量会显著降低"惯性"，让机械臂起步更灵敏，感觉更轻
MASS = 5.0          

# 阻尼 D: 维持 15.0
# 阻尼类似于水的阻力，设太小会导致停不下来，15.0 是个比较稳的值
DAMPING = 80.0      

# 刚度 K: 0.0 (零力拖动模式)
STIFFNESS = 50.0     

# --- 控制参数 ---
CONTROL_RATE = 200.0
DT = 1.0 / CONTROL_RATE

# [修改] 死区阈值: 设为 2.0N (防止零漂导致蠕动)
FORCE_DEADBAND = 2.0

MAX_OFFSET = 0.40
# ===========================================

class AdmittanceController:
    def __init__(self):
        rospy.init_node('aubo_admittance_kdl', anonymous=False)
        
        # 1. 初始化驱动
        self.driver = AuboDriver(use_simulation=USE_SIMULATION, real_ip=REAL_IP)
        if not self.driver.connect():
            sys.exit(1)

        # 2. 初始化 KDL 运动学
        if not self.init_kdl():
            self.driver.disconnect()
            sys.exit(1)

        # 3. 状态初始化
        try:
            self.current_joints = list(self.driver.get_current_joints())
            
            # 计算初始位姿
            self.initial_frame = self.get_fk(self.current_joints)
            # 记录初始位置向量 (x, y, z)
            self.initial_pos = np.array([
                self.initial_frame.p[0],
                self.initial_frame.p[1],
                self.initial_frame.p[2]
            ])
            
            # 导纳状态变量
            self.state_pos_diff = np.zeros(3) 
            self.state_vel = np.zeros(3)      
            
            rospy.loginfo(f"Initial Pos: {self.initial_pos}")
            
        except Exception as e:
            rospy.logerr(f"Init State Failed: {e}")
            self.driver.disconnect()
            sys.exit(1)

        # 4. 力传感器
        self.raw_force = np.zeros(3)
        self.force_bias = np.zeros(3)
        self.force_sub = rospy.Subscriber("/force", WrenchStamped, self.force_callback)
        
        # 可视化
        self.pub_state = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def init_kdl(self):
        """加载 URDF 并构建 KDL求解器"""
        rospy.loginfo("Loading KDL from URDF...")
        success, tree = treeFromFile(URDF_PATH)
        if not success:
            rospy.logerr("Failed to parse URDF")
            return False
            
        self.chain = tree.getChain(BASE_LINK, TIP_LINK)
        self.num_joints = self.chain.getNrOfJoints()
        rospy.loginfo(f"KDL Chain loaded. Joints: {self.num_joints}")
        
        # 创建求解器
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        # 使用 LMA (Levenberg-Marquardt) 数值解法，适合连续运动跟踪
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.chain)
        
        return True

    def get_fk(self, joints):
        """正运动学: 关节角 -> 笛卡尔Frame"""
        q = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints):
            q[i] = joints[i]
        
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(q, frame)
        return frame

    def get_ik(self, target_frame, seed_joints):
        """逆运动学: 目标Frame + 种子关节 -> 目标关节"""
        q_seed = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints):
            q_seed[i] = seed_joints[i]
            
        q_out = PyKDL.JntArray(self.num_joints)
        ret = self.ik_solver.CartToJnt(q_seed, target_frame, q_out)
        
        if ret >= 0:
            return [q_out[i] for i in range(self.num_joints)]
        else:
            return None

    def force_callback(self, msg):
        self.raw_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def calibrate_sensor(self):
        """
        [自动调零功能]
        启动时采样 100 次，计算平均值作为零点偏差。
        """
        print("\n" + "="*50)
        print(">>> 正在执行自动调零 (采样2秒)... <<<")
        print(">>> [警告] 请立刻松手，不要触碰机械臂！ <<<")
        print("="*50)
        
        buffer = []
        for _ in range(100):
            buffer.append(self.raw_force)
            time.sleep(0.02)
        
        self.force_bias = np.mean(buffer, axis=0)
        
        # 已修复 Python 2 到 Python 3 的 print 语法和格式化问题
        print(f"调零完成。零点偏差 (Bias): {self.force_bias}")
        print(f"可以开始拖动了！(死区: +/- {FORCE_DEADBAND}N)\n")

    def get_processed_force(self):
        # 1. 去皮 (扣除零点偏差)
        f_sensor = self.raw_force - self.force_bias
        
        # 2. 死区 (忽略小于 2N 的力)
        if np.linalg.norm(f_sensor) < FORCE_DEADBAND:
            return np.zeros(3)
        
        # 3. 坐标转换
        # 获取当前姿态旋转矩阵
        curr_frame = self.get_fk(self.current_joints)
        
        # KDL Frame 的 Rotation 转换为 numpy 3x3
        # R = [ [M[0,0], M[0,1], M[0,2]], ... ]
        R = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                R[i, j] = curr_frame.M[i, j]
                
        # F_base = R * F_sensor
        f_base = np.dot(R, f_sensor)
        return f_base

    def update_physics(self, f_ext):
        # M*a + D*v + K*x = F
        acc = (f_ext - DAMPING * self.state_vel - STIFFNESS * self.state_pos_diff) / MASS
        
        self.state_vel += acc * DT
        self.state_pos_diff += self.state_vel * DT
        
        # 软限位
        dist = np.linalg.norm(self.state_pos_diff)
        if dist > MAX_OFFSET:
            self.state_pos_diff = self.state_pos_diff / dist * MAX_OFFSET
            self.state_vel = np.zeros(3)

    def run(self):
        if not self.driver.enable_servo():
            return
            
        try:
            self.driver.set_arrival_ahead_time(0.005)
        except:
            pass

        # 启动时自动执行调零
        self.calibrate_sensor()

        rate = rospy.Rate(CONTROL_RATE)
        print(">>> Admittance Control Started (200Hz) <<<")
        
        try:
            while not rospy.is_shutdown():
                # 1. 获取力
                f_base = self.get_processed_force()
                
                # 2. 物理计算
                self.update_physics(f_base)
                
                # 3. 计算目标Frame
                # 逻辑：目标位置 = 初始位置 + 偏移量
                # 姿态：保持初始姿态 (Initial Rotation) 不变 -> 实现平移拖动
                target_frame = PyKDL.Frame()
                target_frame.M = self.initial_frame.M  # 锁死姿态
                
                # 更新位置
                target_pos_vec = self.initial_pos + self.state_pos_diff
                target_frame.p = PyKDL.Vector(target_pos_vec[0], target_pos_vec[1], target_pos_vec[2])
                
                # 4. 逆解
                target_joints = self.get_ik(target_frame, self.current_joints)
                
                if target_joints:
                    # 5. 执行
                    self.driver.send_joints(target_joints)
                    self.current_joints = target_joints
                    
                    # 6. 可视化
                    js_msg = JointState()
                    js_msg.header.stamp = rospy.Time.now()
                    js_msg.name = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 
                                 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
                    js_msg.position = target_joints
                    self.pub_state.publish(js_msg)
                else:
                    rospy.logwarn("IK Failed")

                rate.sleep()

        except Exception as e:
            rospy.logerr(f"Exception: {e}")
        
        finally:
            print("\n>>> Stopping <<<")
            self.driver.disconnect()

if __name__ == "__main__":
    try:
        ctrl = AdmittanceController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass