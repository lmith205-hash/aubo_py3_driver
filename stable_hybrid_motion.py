#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import math
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# [新增] 引入四元数与矩阵转换工具
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_matrix, quaternion_from_matrix

# ROS KDL
import PyKDL
from kdl_parser_py.urdf import treeFromFile

# 引入通用驱动
from unified_driver import AuboDriver

# ================= 配置区域 =================
USE_SIMULATION = True 
REAL_IP = "100.100.1.10"
URDF_PATH = "/home/lmy/aubo_i12/src/aubo_robot/aubo_description/urdf/aubo_i12.urdf"

# KDL 配置
BASE_LINK = "base_link"
TIP_LINK = "wrist3_Link"

# --- 任务流程配置 ---
# 模式选择: 'line_x', 'line_y', 'circle_center'
MOTION_TYPE = 'circle_center' 

# 1. 速度规划参数 (直线和圆弧通用)
TRAJ_MAX_VEL = 0.05     # XY平面巡航线速度: 5cm/s
TRAJ_ACC     = 0.05     # XY平面线加速度: 2cm/s^2

# 2. 直线参数
LINE_LENGTH  = 0.20     # 长度: 20cm

# 3. 圆弧参数 (用于 circle_center 模式)
FIXED_CENTER_XY = [0.7, 0.00] # 圆心绝对坐标 [x, y]
CIRCLE_ANGLE = 360.0   # 旋转总度数 (支持负数代表反向)

# --- [核心融合] 恒力控制参数 (来自 constant_force_control.py) ---
TARGET_FORCE_Z = 15.0   # 目标接触力 (N)

# [关键参数 1] 导纳参数 (使用高阻尼配置以保证稳定)
MASS = np.array([1.0, 1.0, 5.0]) 
DAMPING = np.array([20.0, 20.0, 1500.0]) 
STIFFNESS = np.array([0.0, 0.0, 0.0])    

# [关键参数 2] 力增益 (软化系数)
FORCE_GAIN = 0.3

# [关键参数 3] 滤波器系数 (0.05~1.0, 越小越平滑但延迟高)
FILTER_ALPHA = 0.8

# Z轴调整速度限制 (防止力控调整过猛)
MAX_Z_ADJUST_VEL = 0.1  

# Z轴绝对限位
MIN_Z_HEIGHT = 0.056     # 虚拟地板
MAX_Z_HEIGHT = 0.70      # 虚拟天花板

# 接触判断阈值 (进入运动状态的条件)
CONTACT_TOLERANCE = 2.0 

# --- 系统参数 ---
CONTROL_RATE = 200.0
DT = 1.0 / CONTROL_RATE
FORCE_DEADBAND = 1.0
# ===========================================

# 低通滤波器类
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
        
        filtered = self.alpha * new_val + (1.0 - self.alpha) * self.prev_val
        self.prev_val = filtered
        return filtered

class StableHybridController(object):
    def __init__(self):
        rospy.init_node('aubo_stable_hybrid', anonymous=False)
        
        self.driver = AuboDriver(use_simulation=USE_SIMULATION, real_ip=REAL_IP)
        if not self.driver.connect():
            sys.exit(1)

        if not self.init_kdl():
            self.driver.disconnect()
            sys.exit(1)

        self.force_filter = LowPassFilter(alpha=FILTER_ALPHA)

        # 状态机: 0=下探找力, 1=XY运动+力控维持, 2=运动结束保持
        self.state = 0 
        self.contact_start_time = 0.0
        
        # [新增] 记录当前旋转角度 (用于姿态同步)
        self.current_delta_theta = 0.0 

        # 状态初始化
        try:
            self.current_joints = list(self.driver.get_current_joints())
            self.initial_frame = self.get_fk(self.current_joints)
            self.initial_pos = np.array([
                self.initial_frame.p[0],
                self.initial_frame.p[1],
                self.initial_frame.p[2]
            ])
            
            # [新增] 提取初始姿态并转换为四元数，作为同步旋转的基准
            start_M = self.initial_frame.M
            start_rot_mat = np.array([
                [start_M[0,0], start_M[0,1], start_M[0,2], 0],
                [start_M[1,0], start_M[1,1], start_M[1,2], 0],
                [start_M[2,0], start_M[2,1], start_M[2,2], 0],
                [0, 0, 0, 1]
            ])
            self.initial_quat = quaternion_from_matrix(start_rot_mat)
            
            self.traj_start_pos = np.copy(self.initial_pos)
            self.force_pos_diff = np.zeros(3) 
            self.state_vel = np.zeros(3)      
            
            # 圆弧插补专用变量
            self.circle_radius = 0.0
            self.circle_start_angle = 0.0
            
            rospy.loginfo("Initial Pos: {:.4f}, {:.4f}, {:.4f}".format(
                self.initial_pos[0], self.initial_pos[1], self.initial_pos[2]))
            
        except Exception as e:
            rospy.logerr("Init failed: {}".format(e))
            self.driver.disconnect()
            sys.exit(1)

        self.raw_force = np.zeros(3)
        self.force_bias = np.zeros(3)
        self.force_sub = rospy.Subscriber("/force", WrenchStamped, self.force_callback)
        self.pub_state = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def init_kdl(self):
        success, tree = treeFromFile(URDF_PATH)
        if not success: return False
        self.chain = tree.getChain(BASE_LINK, TIP_LINK)
        self.num_joints = self.chain.getNrOfJoints()
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
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
        print("\n>>> 正在调零 (2秒)... <<<")
        buffer = []
        for _ in range(100):
            buffer.append(self.raw_force)
            time.sleep(0.02)
        self.force_bias = np.mean(buffer, axis=0)
        print("调零完成: ", self.force_bias)

    def get_processed_force(self):
        f_sensor = self.raw_force - self.force_bias
        f_filtered = self.force_filter.update(f_sensor)
        
        if np.linalg.norm(f_filtered) < FORCE_DEADBAND:
            f_filtered = np.zeros(3)
            
        curr_frame = self.get_fk(self.current_joints)
        R = np.zeros((3, 3))
        for i in range(3):
            for j in range(3):
                R[i, j] = curr_frame.M[i, j]
        return np.dot(R, f_filtered)

    def update_force_control(self, f_ext):
        f_target = np.array([0.0, 0.0, TARGET_FORCE_Z])
        force_error = f_ext - f_target
        force_input_softened = force_error * FORCE_GAIN
        
        acc = (force_input_softened - DAMPING * self.state_vel - STIFFNESS * self.force_pos_diff) / MASS
        self.state_vel += acc * DT
        
        self.state_vel[0] = 0.0
        self.state_vel[1] = 0.0
        self.state_vel[2] = np.clip(self.state_vel[2], -MAX_Z_ADJUST_VEL, MAX_Z_ADJUST_VEL)
        
        self.force_pos_diff += self.state_vel * DT
        current_abs_z = self.initial_pos[2] + self.force_pos_diff[2]
        
        if current_abs_z < MIN_Z_HEIGHT:
            self.force_pos_diff[2] = MIN_Z_HEIGHT - self.initial_pos[2]
            if self.state_vel[2] < 0: self.state_vel[2] = 0.0
        elif current_abs_z > MAX_Z_HEIGHT:
            self.force_pos_diff[2] = MAX_Z_HEIGHT - self.initial_pos[2]
            if self.state_vel[2] > 0: self.state_vel[2] = 0.0
            
        return force_error[2]

    def calculate_trapezoidal_dist(self, t, total_dist):
        dist_mag = abs(total_dist)
        sign = 1.0 if total_dist >= 0 else -1.0
        
        t_acc = TRAJ_MAX_VEL / TRAJ_ACC
        d_acc = 0.5 * TRAJ_ACC * t_acc * t_acc
        
        if 2 * d_acc > dist_mag: 
            d_acc = dist_mag / 2.0
            t_acc = math.sqrt(2 * d_acc / TRAJ_ACC)
            t_flat = 0.0
        else: 
            d_flat = dist_mag - 2 * d_acc
            t_flat = d_flat / TRAJ_MAX_VEL
            
        total_time = 2 * t_acc + t_flat
        
        s = 0.0
        is_finished = False
        
        if t <= t_acc:
            s = 0.5 * TRAJ_ACC * t * t
        elif t <= (t_acc + t_flat):
            dt = t - t_acc
            s = d_acc + TRAJ_MAX_VEL * dt
        elif t <= total_time:
            t_dec = t - (t_acc + t_flat)
            t_remain = t_acc - t_dec
            s = dist_mag - 0.5 * TRAJ_ACC * t_remain * t_remain
        else:
            s = dist_mag
            is_finished = True
            
        return s * sign, is_finished

    def get_trajectory_pos(self, t):
        ref_pos = np.copy(self.traj_start_pos)
        is_finished = False
        delta_theta = 0.0  # [新增] 记录偏航角的变化量
        
        if 'line' in MOTION_TYPE:
            dist, is_finished = self.calculate_trapezoidal_dist(t, LINE_LENGTH)
            if MOTION_TYPE == 'line_x': ref_pos[0] += dist
            elif MOTION_TYPE == 'line_y': ref_pos[1] += dist
                
        elif MOTION_TYPE == 'circle_center':
            total_arc_len = self.circle_radius * math.radians(CIRCLE_ANGLE)
            current_arc_len, is_finished = self.calculate_trapezoidal_dist(t, total_arc_len)
            
            # [核心修改] 实时计算出已转过的弧度 (delta_theta)
            delta_theta = current_arc_len / self.circle_radius
            current_theta = self.circle_start_angle + delta_theta
            
            ref_pos[0] = FIXED_CENTER_XY[0] + self.circle_radius * math.cos(current_theta)
            ref_pos[1] = FIXED_CENTER_XY[1] + self.circle_radius * math.sin(current_theta)
            
        return ref_pos, is_finished, delta_theta

    def run(self):
        if not self.driver.enable_servo(): return
        try: self.driver.set_arrival_ahead_time(0.005)
        except: pass

        self.calibrate_sensor()

        print("\n>>> 稳定版混合控制启动 (姿态同步开启) <<<")
        print(">>> 阶段1: 下探找力 (目标:{}N, 滤波器:{}, 增益:{})".format(TARGET_FORCE_Z, FILTER_ALPHA, FORCE_GAIN))
        
        if MOTION_TYPE == 'circle_center':
            print(">>> 阶段2: 圆心插补 (圆心: {}, 角度: {:.1f}度)".format(FIXED_CENTER_XY, CIRCLE_ANGLE))
        else:
            print(">>> 阶段2: 直线插补 (模式: {}, 长度: {}m)".format(MOTION_TYPE, LINE_LENGTH))
        
        rate = rospy.Rate(CONTROL_RATE)
        motion_time_start = 0.0
        
        # 预先分配轨迹坐标
        traj_pos = np.copy(self.initial_pos)
        
        try:
            while not rospy.is_shutdown():
                # 1. 测力与物理引擎修正
                f_base = self.get_processed_force()
                z_force_error = self.update_force_control(f_base)
                current_abs_z = self.initial_pos[2] + self.force_pos_diff[2]
                
                # 2. 状态机逻辑
                if self.state == 0: # 寻找接触
                    traj_pos = np.copy(self.initial_pos)
                    self.current_delta_theta = 0.0 # 重置角度
                    
                    is_force_stable = abs(z_force_error) < CONTACT_TOLERANCE
                    is_bottomed_out = current_abs_z <= MIN_Z_HEIGHT + 0.002

                    if is_force_stable or is_bottomed_out:
                        if is_bottomed_out or (time.time() - self.contact_start_time > 1.0):
                            print("\n>>> [事件] 接触稳定！开始XY平面运动...")
                            self.traj_start_pos = np.copy(self.initial_pos)
                            
                            if MOTION_TYPE == 'circle_center':
                                dx = self.traj_start_pos[0] - FIXED_CENTER_XY[0]
                                dy = self.traj_start_pos[1] - FIXED_CENTER_XY[1]
                                self.circle_radius = math.sqrt(dx*dx + dy*dy)
                                self.circle_start_angle = math.atan2(dy, dx)
                                print("    - 半径 R = {:.4f} m".format(self.circle_radius))
                            
                            motion_time_start = time.time()
                            self.state = 1
                        else:
                            if self.contact_start_time == 0.0: self.contact_start_time = time.time()
                    else:
                        self.contact_start_time = 0.0

                elif self.state == 1: # 执行XY运动
                    t_motion = time.time() - motion_time_start
                    # [接收角度]
                    traj_pos, finished, self.current_delta_theta = self.get_trajectory_pos(t_motion)
                    
                    if finished:
                        print("\n>>> [事件] 轨迹运行完毕。")
                        self.state = 2 

                elif self.state == 2: # 保持原位
                    pass
                
                # 3. 构造 IK 逆解的最终坐标和姿态
                target_pos_vec = np.copy(traj_pos)
                target_pos_vec[2] = self.initial_pos[2] + self.force_pos_diff[2]
                
                target_frame = PyKDL.Frame()
                target_frame.p = PyKDL.Vector(target_pos_vec[0], target_pos_vec[1], target_pos_vec[2])
                
                # [核心姿态同步计算]：在初始姿态基础上，叠加 Z 轴旋转 (delta_theta)
                q_rot = quaternion_from_euler(0, 0, self.current_delta_theta)
                curr_quat = quaternion_multiply(q_rot, self.initial_quat)
                mat = quaternion_matrix(curr_quat)
                
                target_frame.M = PyKDL.Rotation(mat[0,0], mat[0,1], mat[0,2],
                                                mat[1,0], mat[1,1], mat[1,2],
                                                mat[2,0], mat[2,1], mat[2,2])
                
                # 4. 发送指令
                target_joints = self.get_ik(target_frame, self.current_joints)
                if target_joints:
                    self.driver.send_joints(target_joints)
                    self.current_joints = target_joints
                    
                    js_msg = JointState()
                    js_msg.header.stamp = rospy.Time.now()
                    js_msg.name = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
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
        ctrl = StableHybridController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
