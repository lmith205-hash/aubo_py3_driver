#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import math
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# ROS KDL 库
import PyKDL
from kdl_parser_py.urdf import treeFromFile

# 引入通用驱动
from unified_driver import AuboDriver

# ================= 配置区域 =================
USE_SIMULATION = True   # True: 仿真; False: 真机
REAL_IP = "100.100.1.10"
URDF_PATH = "/home/lmy/aubo_i12/src/aubo_robot/aubo_robot/aubo_description/urdf/aubo_i12.urdf"

# KDL 链条配置
BASE_LINK = "base_link"
TIP_LINK = "wrist3_Link"

# --- 恒力控制参数 ---
TARGET_FORCE_Z = 15.0  # 目标恒力

# [参数优化] 保持较大质量和阻尼
MASS = np.array([1.0, 1.0, 15.0]) 
DAMPING = np.array([20.0, 20.0, 1200.0])
STIFFNESS = np.array([0.0, 0.0, 0.0])

# [软化系数] 
FORCE_GAIN = 0.3

# --- [新增] 滤波器系数 ---
# 范围 0.0 ~ 1.0。越小越平滑，但延迟越大。
# 0.05 意味着新数据只占 5% 权重，历史数据占 95%，滤除高频噪声效果极好
FILTER_ALPHA = 0.8

# --- 控制参数 ---
CONTROL_RATE = 200.0
DT = 1.0 / CONTROL_RATE
FORCE_DEADBAND = 1.0   

# --- 安全边界 ---
MIN_Z_HEIGHT = 0.056    
MAX_Z_RISE = 0.50      
# [优化] 进一步降低最大速度，防止调节过猛
MAX_VELOCITY = 0.1    
# ===========================================

# [新增] 简单的低通滤波器类
class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.prev_val = np.zeros(3)
        self.first_run = True

    def update(self, new_val):
        if self.first_run:
            self.prev_val = new_val
            self.first_run = False
            return new_val
        
        # 滤波公式: out = alpha * new + (1-alpha) * old
        filtered = self.alpha * new_val + (1.0 - self.alpha) * self.prev_val
        self.prev_val = filtered
        return filtered

class ConstantForceController(object):
    def __init__(self):
        rospy.init_node('aubo_constant_force', anonymous=False)
        
        # 1. 初始化驱动
        self.driver = AuboDriver(use_simulation=USE_SIMULATION, real_ip=REAL_IP)
        if not self.driver.connect():
            sys.exit(1)

        # 2. KDL
        if not self.init_kdl():
            self.driver.disconnect()
            sys.exit(1)

        # [新增] 初始化滤波器
        self.force_filter = LowPassFilter(alpha=FILTER_ALPHA)

        # 3. 状态初始化
        try:
            self.current_joints = list(self.driver.get_current_joints())
            
            # 记录初始位姿
            self.initial_frame = self.get_fk(self.current_joints)
            self.initial_pos = np.array([
                self.initial_frame.p[0],
                self.initial_frame.p[1],
                self.initial_frame.p[2]
            ])
            
            # 状态变量
            self.state_pos_diff = np.zeros(3) 
            self.state_vel = np.zeros(3)      
            
            rospy.loginfo("Initial Pos: {:.4f}, {:.4f}, {:.4f}".format(
                self.initial_pos[0], self.initial_pos[1], self.initial_pos[2]))
            
            if self.initial_pos[2] < MIN_Z_HEIGHT:
                rospy.logwarn("警告！当前高度 ({:.4f}) 低于设定的虚拟地板 ({:.4f})!".format(
                    self.initial_pos[2], MIN_Z_HEIGHT))
            
        except Exception as e:
            rospy.logerr("Init failed: {}".format(e))
            self.driver.disconnect()
            sys.exit(1)

        # 4. 传感器
        self.raw_force = np.zeros(3)
        self.force_bias = np.zeros(3)
        self.force_sub = rospy.Subscriber("/force", WrenchStamped, self.force_callback)
        self.pub_state = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def init_kdl(self):
        rospy.loginfo("Loading KDL...")
        success, tree = treeFromFile(URDF_PATH)
        if not success: return False
        self.chain = tree.getChain(BASE_LINK, TIP_LINK)
        self.num_joints = self.chain.getNrOfJoints()
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        # 保持使用 LMA
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.chain)
        return True

    def get_fk(self, joints):
        q = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints): q[i] = joints[i]
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(q, frame)
        return frame

    def get_ik(self, target_frame, seed_joints):
        q_seed = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints): q_seed[i] = seed_joints[i]
        q_out = PyKDL.JntArray(self.num_joints)
        ret = self.ik_solver.CartToJnt(q_seed, target_frame, q_out)
        return [q_out[i] for i in range(self.num_joints)] if ret >= 0 else None

    def force_callback(self, msg):
        self.raw_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])

    def calibrate_sensor(self):
        print("\n" + "="*50)
        print(">>> 正在调零 (2秒) <<<")
        print(">>> 此时请保持悬空，不要接触任何物体！ <<<")
        print("="*50)
        buffer = []
        for _ in range(100):
            buffer.append(self.raw_force)
            time.sleep(0.02)
        self.force_bias = np.mean(buffer, axis=0)
        print("调零完成。零偏: ", self.force_bias)

    def get_processed_force(self):
        # 1. 去皮
        f_sensor = self.raw_force - self.force_bias
        
        # [修改] 2. 先应用低通滤波，再做死区判断
        # 这样可以防止噪声在死区边缘反复横跳
        f_filtered = self.force_filter.update(f_sensor)
        
        # 3. 死区
        if np.linalg.norm(f_filtered) < FORCE_DEADBAND:
            f_filtered = np.zeros(3)
            
        # 4. 坐标系转换 (Sensor Frame -> Base Frame)
        curr_frame = self.get_fk(self.current_joints)
        R = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                R[i, j] = curr_frame.M[i, j]
        
        f_base = np.dot(R, f_filtered)
        return f_base

    def update_physics(self, f_ext):
        f_target = np.array([0.0, 0.0, TARGET_FORCE_Z])
        force_error = f_ext - f_target
        
        force_input_softened = force_error * FORCE_GAIN
        
        acc = (force_input_softened - DAMPING * self.state_vel - STIFFNESS * self.state_pos_diff) / MASS
        
        self.state_vel += acc * DT
        self.state_vel[0] = 0.0
        self.state_vel[1] = 0.0
        # [修改] 使用新的更低限速
        self.state_vel[2] = np.clip(self.state_vel[2], -MAX_VELOCITY, MAX_VELOCITY)
        
        self.state_pos_diff += self.state_vel * DT
        self.state_pos_diff[0] = 0.0
        self.state_pos_diff[1] = 0.0
        
        current_abs_z = self.initial_pos[2] + self.state_pos_diff[2]
        
        if current_abs_z < MIN_Z_HEIGHT:
            self.state_pos_diff[2] = MIN_Z_HEIGHT - self.initial_pos[2]
            if self.state_vel[2] < 0:
                self.state_vel[2] = 0.0
        elif self.state_pos_diff[2] > MAX_Z_RISE:
            self.state_pos_diff[2] = MAX_Z_RISE
            if self.state_vel[2] > 0:
                self.state_vel[2] = 0.0

    def run(self):
        if not self.driver.enable_servo():
            return
        
        try: self.driver.set_arrival_ahead_time(0.005)
        except: pass

        self.calibrate_sensor()

        rate = rospy.Rate(CONTROL_RATE)
        print("\n>>> 恒力控制 (滤波稳定版) 已启动 <<<")
        print(">>> 滤波器 Alpha: {} (值越小越平滑) <<<".format(FILTER_ALPHA))
        
        try:
            while not rospy.is_shutdown():
                f_base = self.get_processed_force()
                self.update_physics(f_base)
                
                target_frame = PyKDL.Frame()
                target_frame.M = self.initial_frame.M
                
                reference_pos = self.initial_pos
                target_pos_vec = reference_pos + self.state_pos_diff
                target_frame.p = PyKDL.Vector(target_pos_vec[0], target_pos_vec[1], target_pos_vec[2])
                
                target_joints = self.get_ik(target_frame, self.current_joints)
                
                if target_joints:
                    self.driver.send_joints(target_joints)
                    self.current_joints = target_joints
                    
                    js_msg = JointState()
                    js_msg.header.stamp = rospy.Time.now()
                    js_msg.name = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 
                                 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
                    js_msg.position = target_joints
                    self.pub_state.publish(js_msg)
                else:
                    rospy.logwarn_throttle(2.0, "IK Failed")

                rate.sleep()

        except Exception as e:
            rospy.logerr("Error: {}".format(e))
        
        finally:
            print("\n>>> 停止 <<<")
            self.driver.disconnect()

if __name__ == "__main__":
    try:
        ctrl = ConstantForceController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass