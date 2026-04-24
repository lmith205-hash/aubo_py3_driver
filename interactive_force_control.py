#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import time
import math
import numpy as np
import threading
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

# ROS KDL
import PyKDL
from kdl_parser_py.urdf import treeFromFile

# 引入通用驱动
from unified_driver import AuboDriver

# ================= 配置区域 =================
USE_SIMULATION = False 
REAL_IP = "100.100.1.10"
URDF_PATH = "/home/lmy/aubo_i12/src/aubo_robot/aubo_robot/aubo_description/urdf/aubo_i12.urdf"

# KDL 配置
BASE_LINK = "base_link"
TIP_LINK = "wrist3_Link"

# --- 运动参数 ---
TRAJ_MAX_VEL = 0.03     # XY平面最大速度: 3cm/s
TRAJ_ACC     = 0.05     # XY平面加速度: 5cm/s^2

# --- 恒力控制参数 ---
TARGET_FORCE_Z = 15.0   # 目标接触力 (N)
MASS = np.array([1.0, 1.0, 5.0]) 
DAMPING = np.array([20.0, 20.0, 1500.0]) 
STIFFNESS = np.array([0.0, 0.0, 0.0])    
FORCE_GAIN = 0.3        # 力误差增益
FILTER_ALPHA = 0.8      # 滤波系数
MAX_Z_ADJUST_VEL = 0.1  # Z轴调整速度限制

# Z轴绝对限位 (保护)
MIN_Z_HEIGHT = 0.056     # 虚拟地板
MAX_Z_HEIGHT = 0.70      # 虚拟天花板

# 接触判断阈值
CONTACT_TOLERANCE = 2.0 
FORCE_DEADBAND = 1.0

# 系统频率
CONTROL_RATE = 200.0
DT = 1.0 / CONTROL_RATE
# ===========================================

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

class InteractiveHybridController(object):
    def __init__(self):
        rospy.init_node('aubo_interactive_force', anonymous=False)
        
        self.driver = AuboDriver(use_simulation=USE_SIMULATION, real_ip=REAL_IP)
        if not self.driver.connect():
            sys.exit(1)

        if not self.init_kdl():
            self.driver.disconnect()
            sys.exit(1)

        self.force_filter = LowPassFilter(alpha=FILTER_ALPHA)
        
        # 线程通信变量
        self.input_lock = threading.Lock()
        self.new_target_xy = None # (x, y)
        self.is_input_active = False

        # 状态变量
        self.state = 0 # 0=下探找力, 1=接触保持(空闲), 2=移动中
        self.contact_start_time = 0.0
        
        # 运动规划变量
        self.current_xy_ref = np.zeros(2) # 当前XY参考位置
        self.start_xy = np.zeros(2)       # 移动起始点
        self.target_xy = np.zeros(2)      # 移动目标点
        self.motion_start_time = 0.0
        self.motion_duration = 0.0

        try:
            self.current_joints = list(self.driver.get_current_joints())
            self.initial_frame = self.get_fk(self.current_joints)
            
            # 记录初始位置 (Z轴基准)
            self.initial_pos = np.array([
                self.initial_frame.p[0],
                self.initial_frame.p[1],
                self.initial_frame.p[2]
            ])
            
            # 初始化 XY 参考位置
            self.current_xy_ref = self.initial_pos[0:2]

            self.force_pos_diff = np.zeros(3) 
            self.state_vel = np.zeros(3)      
            
            rospy.loginfo("Init Pos: {:.4f}, {:.4f}, {:.4f}".format(
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
        print("(\n>>> 正在调零 (2秒)... <<<)")
        buffer = []
        for _ in range(100):
            buffer.append(self.raw_force)
            time.sleep(0.02)
        self.force_bias = np.mean(buffer, axis=0)
        print("(调零完成: {})".format(self.force_bias))

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
        """ 力控核心计算: 计算 Z 轴的位置修正量 """
        f_target = np.array([0.0, 0.0, TARGET_FORCE_Z])
        force_error = f_ext - f_target
        
        force_input_softened = force_error * FORCE_GAIN
        acc = (force_input_softened - DAMPING * self.state_vel - STIFFNESS * self.force_pos_diff) / MASS
        
        self.state_vel += acc * DT
        self.state_vel[0] = 0.0 # XY速度由位置规划控制
        self.state_vel[1] = 0.0
        self.state_vel[2] = np.clip(self.state_vel[2], -MAX_Z_ADJUST_VEL, MAX_Z_ADJUST_VEL)
        
        self.force_pos_diff += self.state_vel * DT
        
        # 限位处理
        current_abs_z = self.initial_pos[2] + self.force_pos_diff[2]
        if current_abs_z < MIN_Z_HEIGHT:
            self.force_pos_diff[2] = MIN_Z_HEIGHT - self.initial_pos[2]
            if self.state_vel[2] < 0: self.state_vel[2] = 0.0
        elif current_abs_z > MAX_Z_HEIGHT:
            self.force_pos_diff[2] = MAX_Z_HEIGHT - self.initial_pos[2]
            if self.state_vel[2] > 0: self.state_vel[2] = 0.0
            
        return force_error[2]

    # --- 输入处理线程 ---
    def input_worker(self):
        """ 独立线程处理用户输入，不阻塞主控制循环 """
        print("(\n" + "="*40 + ")")
        print("(   交互模式已启动)")
        print("(   请输入目标 XY 坐标，格式: x y)")
        print("(   例如: 0.65 -0.1)")
        print("(   输入 'q' 退出)")
        print("(" + "="*40 + "\n)")
        
        while not rospy.is_shutdown() and self.is_input_active:
            try:
                # 兼容 Python 2 和 Python 3
                if sys.version_info[0] < 3:
                    user_str = raw_input(">>> 请输入目标坐标 (x y): ")
                else:
                    user_str = input(">>> 请输入目标坐标 (x y): ")
                
                user_str = user_str.strip()
                if user_str.lower() == 'q':
                    rospy.signal_shutdown("User Quit")
                    break
                
                parts = user_str.replace(',', ' ').split()
                if len(parts) >= 2:
                    tx = float(parts[0])
                    ty = float(parts[1])
                    
                    with self.input_lock:
                        self.new_target_xy = np.array([tx, ty])
                    
                    print("(--> 指令已接收: 目标 X={:.3f}, Y={:.3f})".format(tx, ty))
                else:
                    print("(!! 格式错误，请重新输入。正确示例: 0.6 -0.1)")
            
            except ValueError:
                print("(!! 数值无效)")
            except Exception as e:
                # 避免退出时报错干扰
                if not rospy.is_shutdown():
                    print("(!! 输入错误: {})".format(e))
    
    # --- 运动规划 ---
    def plan_motion(self, start_xy, end_xy):
        """ 计算移动所需时间和距离 """
        dist = np.linalg.norm(end_xy - start_xy)
        if dist < 0.001:
            return 0.0 # 距离太小，不移动
        
        # 简单梯形规划估算时间 (加速+匀速+减速)
        # T = (Distance / Vel) + (Vel / Acc) approx
        t_acc = TRAJ_MAX_VEL / TRAJ_ACC
        d_acc = 0.5 * TRAJ_ACC * t_acc**2
        
        if 2 * d_acc > dist:
            # 三角形规划 (达不到最大速度)
            total_time = 2 * math.sqrt(dist / TRAJ_ACC)
        else:
            # 梯形规划
            d_const = dist - 2 * d_acc
            t_const = d_const / TRAJ_MAX_VEL
            total_time = 2 * t_acc + t_const
            
        return total_time

    def get_interpolated_pos(self, t):
        """ 根据当前时间 t 获取 XY 插补位置 """
        if self.motion_duration <= 0.0001:
            return self.target_xy, True
            
        progress = t / self.motion_duration
        if progress >= 1.0:
            return self.target_xy, True
        
        # 使用平滑的 S 曲线或简单的线性插值，这里使用简单的比例插值
        # 也可以复用原来的梯形速度公式，为了简单稳定，这里用 sin 缓动
        # Ease-in-out: 0.5 * (1 - cos(pi * p))
        alpha = 0.5 * (1.0 - math.cos(math.pi * progress))
        
        curr_pos = self.start_xy + (self.target_xy - self.start_xy) * alpha
        return curr_pos, False

    def run(self):
        if not self.driver.enable_servo(): return
        self.calibrate_sensor()

        print("(\n>>> 控制器启动 <<<)")
        print("(阶段 1: 正在下探寻找接触面 (保持力: {}N)...)".format(TARGET_FORCE_Z))
        
        rate = rospy.Rate(CONTROL_RATE)
        
        try:
            while not rospy.is_shutdown():
                # 1. 实时力控计算 (Z轴)
                f_base = self.get_processed_force()
                z_force_error = self.update_force_control(f_base)
                current_abs_z = self.initial_pos[2] + self.force_pos_diff[2]

                # ================= 状态机 =================
                
                # --- 状态 0: 下探找力 ---
                if self.state == 0:
                    is_stable = abs(z_force_error) < CONTACT_TOLERANCE
                    is_bottom = current_abs_z <= MIN_Z_HEIGHT + 0.002
                    
                    if is_stable or is_bottom:
                        if is_bottom or (time.time() - self.contact_start_time > 1.5):
                            print("(\n>>> [事件] 接触已建立且稳定！)")
                            print("(>>> [事件] 启动交互输入线程...)")
                            
                            # 启动输入线程
                            self.is_input_active = True
                            t = threading.Thread(target=self.input_worker)
                            t.daemon = True
                            t.start()
                            
                            # 更新当前参考位置为实际位置
                            curr_frame = self.get_fk(self.current_joints)
                            self.current_xy_ref = np.array([curr_frame.p[0], curr_frame.p[1]])
                            
                            self.state = 1 # 转入空闲等待状态
                        else:
                            if self.contact_start_time == 0.0: self.contact_start_time = time.time()
                    else:
                        self.contact_start_time = 0.0

                # --- 状态 1: 空闲维持 (等待指令) ---
                elif self.state == 1:
                    # 检查是否有新指令
                    with self.input_lock:
                        if self.new_target_xy is not None:
                            self.start_xy = np.copy(self.current_xy_ref)
                            self.target_xy = np.copy(self.new_target_xy)
                            self.new_target_xy = None # 清除指令
                            
                            # 规划
                            self.motion_duration = self.plan_motion(self.start_xy, self.target_xy)
                            if self.motion_duration > 0:
                                self.motion_start_time = time.time()
                                self.state = 2 # 开始移动
                                print("(>>> [移动] 开始移动到 {}, 预计耗时 {:.2f}s)".format(self.target_xy, self.motion_duration))
                            else:
                                print("(>>> [警告] 目标距离过近，忽略)")

                # --- 状态 2: 移动中 ---
                elif self.state == 2:
                    t_elapsed = time.time() - self.motion_start_time
                    next_xy, finished = self.get_interpolated_pos(t_elapsed)
                    
                    self.current_xy_ref = next_xy
                    
                    if finished:
                        print("(>>> [完成] 到达目标点。保持力控，等待新坐标...)")
                        self.state = 1 # 回到空闲状态

                # ============================================

                # 3. 组合最终目标位置
                # XY来自轨迹规划(或保持)，Z来自力控修正
                target_pos_vec = np.zeros(3)
                target_pos_vec[0] = self.current_xy_ref[0]
                target_pos_vec[1] = self.current_xy_ref[1]
                # Z = 初始参考Z + 力控动态调整量
                target_pos_vec[2] = self.initial_pos[2] + self.force_pos_diff[2]
                
                # 4. 逆解与执行
                target_frame = PyKDL.Frame()
                target_frame.M = self.initial_frame.M # 保持姿态不变
                target_frame.p = PyKDL.Vector(target_pos_vec[0], target_pos_vec[1], target_pos_vec[2])
                
                target_joints = self.get_ik(target_frame, self.current_joints)
                
                if target_joints:
                    self.driver.send_joints(target_joints)
                    self.current_joints = target_joints
                    
                    # 发布关节状态供Rviz显示
                    js_msg = JointState()
                    js_msg.header.stamp = rospy.Time.now()
                    js_msg.name = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
                    js_msg.position = target_joints
                    self.pub_state.publish(js_msg)
                else:
                    # 逆解失败保护：不移动
                    pass

                rate.sleep()

        except Exception as e:
            rospy.logerr("Error: {}".format(e))
        
        finally:
            self.is_input_active = False # 停止输入线程
            print("(\n>>> 程序结束 <<<)")
            self.driver.disconnect()

if __name__ == "__main__":
    try:
        ctrl = InteractiveHybridController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass