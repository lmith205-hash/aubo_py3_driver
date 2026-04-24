#!/usr/bin/env python
# -*- coding: utf-8 -*-

try:
    input = raw_input
except NameError:
    pass

import rospy
import sys
import time
import math
import numpy as np
from sensor_msgs.msg import JointState

# ROS KDL
import PyKDL
from kdl_parser_py.urdf import treeFromFile

# 引入通用驱动
from unified_driver import AuboDriver

# ================= 配置区域 =================
USE_SIMULATION =False  # 是否使用仿真环境，False表示连接真机   
REAL_IP = "100.100.1.10"
URDF_PATH = "/home/lmy/aubo_i12/src/aubo_robot/aubo_robot/aubo_description/urdf/aubo_i12.urdf"

BASE_LINK = "base_link"
TIP_LINK = "wrist3_Link"

# 运动参数 (梯形速度规划)
MAX_VELOCITY = 0.05     # 最大巡航速度: 5cm/s
ACCELERATION = 0.02     # 加速度: 2cm/s^2 (约2.5秒加速到最大速度)
CONTROL_RATE = 200.0    # 200Hz
# ===========================================

class LowPassFilter(object):
    """
    关节角度低通滤波器
    公式: Y_new = alpha * X_new + (1 - alpha) * Y_old
    """
    def __init__(self, num_joints, alpha=0.15):
        self.alpha = alpha
        self.num_joints = num_joints
        self.last_cmd = None # 存储上一帧平滑后的角度

    def filter(self, new_cmd):
        # 如果是第一帧数据，直接透传并初始化
        if self.last_cmd is None:
            self.last_cmd = new_cmd
            return new_cmd
        
        filtered_cmd = []
        for i in range(self.num_joints):
            # 平滑计算
            val = self.alpha * new_cmd[i] + (1.0 - self.alpha) * self.last_cmd[i]
            filtered_cmd.append(val)
            
        self.last_cmd = filtered_cmd
        return filtered_cmd

class CartesianController(object):
    def __init__(self):
        rospy.init_node('aubo_linear_trap', anonymous=False)
        
        self.driver = AuboDriver(use_simulation=USE_SIMULATION, real_ip=REAL_IP)
        if not self.driver.connect():
            sys.exit(1)

        if not self.init_kdl():
            self.driver.disconnect()
            sys.exit(1)

        try:
            self.current_joints = list(self.driver.get_current_joints())
            self.pub_state = rospy.Publisher("/joint_states", JointState, queue_size=10)
            
            if not self.driver.enable_servo():
                rospy.logerr("无法开启透传模式")
                sys.exit(1)
                
            try: self.driver.set_arrival_ahead_time(0.005)
            except: pass
            
        except Exception as e:
            rospy.logerr("初始化失败: {}".format(e))
            self.driver.disconnect()
            sys.exit(1)

    def init_kdl(self):
        success, tree = treeFromFile(URDF_PATH)
        if not success: return False
        self.chain = tree.getChain(BASE_LINK, TIP_LINK)
        self.num_joints = self.chain.getNrOfJoints()
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(self.chain)
        return True

    def get_current_pose(self):
        self.current_joints = list(self.driver.get_current_joints())
        q = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints): q[i] = self.current_joints[i]
        frame = PyKDL.Frame()
        self.fk_solver.JntToCart(q, frame)
        return frame

    def get_ik(self, target_frame, seed_joints):
        q_seed = PyKDL.JntArray(self.num_joints)
        for i in range(self.num_joints): q_seed[i] = seed_joints[i]
        q_out = PyKDL.JntArray(self.num_joints)
        ret = self.ik_solver.CartToJnt(q_seed, target_frame, q_out)
        return [q_out[i] for i in range(self.num_joints)] if ret >= 0 else None

    def move_linear_trap(self, target_x, target_y, target_z):
        """
        执行梯形速度规划 (Trapezoidal Velocity Profile)
        """
        start_frame = self.get_current_pose()
        start_pos = np.array([start_frame.p[0], start_frame.p[1], start_frame.p[2]])
        target_pos = np.array([target_x, target_y, target_z])
        
        vector = target_pos - start_pos
        total_distance = np.linalg.norm(vector)
        
        if total_distance < 0.001:
            print(">>> 目标太近。")
            return

        # --- 规划计算 (保持不变) ---
        t_acc = MAX_VELOCITY / ACCELERATION
        d_acc = 0.5 * ACCELERATION * t_acc * t_acc
        
        if 2 * d_acc > total_distance:
            d_acc = total_distance / 2.0
            t_acc = math.sqrt(2 * d_acc / ACCELERATION)
            t_flat = 0.0
            v_peak = ACCELERATION * t_acc 
            print(">>> 短距离移动 (三角形规划): 峰值速度 {:.3f}m/s".format(v_peak))
        else:
            d_flat = total_distance - 2 * d_acc
            t_flat = d_flat / MAX_VELOCITY
            print(">>> 长距离移动 (梯形规划): 巡航速度 {:.3f}m/s".format(MAX_VELOCITY))

        total_time = 2 * t_acc + t_flat
        print(">>> 预计耗时: {:.2f}s".format(total_time))
        
        rate = rospy.Rate(CONTROL_RATE)
        start_time = time.time()
        
        # [修改点 1] 初始化滤波器 (alpha=0.15 比较稳妥)
        # alpha越小越平滑但延迟越高。0.1 ~ 0.2 是推荐范围。
        lpf = LowPassFilter(self.num_joints, alpha=0.15)
        
        while not rospy.is_shutdown():
            t = time.time() - start_time
            
            if t > total_time:
                break # 到达终点
            
            # --- 计算当前位移 s(t) ---
            current_dist = 0.0
            
            if t <= t_acc:
                current_dist = 0.5 * ACCELERATION * t * t
            elif t <= (t_acc + t_flat):
                dt_flat = t - t_acc
                current_dist = d_acc + MAX_VELOCITY * dt_flat
            else:
                t_dec = t - (t_acc + t_flat)
                t_remain = t_acc - t_dec
                current_dist = total_distance - 0.5 * ACCELERATION * t_remain * t_remain

            ratio = current_dist / total_distance
            if ratio > 1.0: ratio = 1.0
            
            current_target_vec = start_pos + vector * ratio
            
            # 逆解
            target_frame = PyKDL.Frame()
            target_frame.M = start_frame.M
            target_frame.p = PyKDL.Vector(current_target_vec[0], current_target_vec[1], current_target_vec[2])
            
            target_joints = self.get_ik(target_frame, self.current_joints)
            
            if target_joints:
                # [修改点 2] 应用滤波器
                smooth_joints = lpf.filter(target_joints)
                
                # 发送平滑后的数据
                self.driver.send_joints(smooth_joints)
                
                # 更新当前关节 (用于下一次 IK 的种子)
                # 使用平滑后的值作为种子通常更稳定
                self.current_joints = smooth_joints
                
                js_msg = JointState()
                js_msg.header.stamp = rospy.Time.now()
                js_msg.name = ['shoulder_joint', 'upperArm_joint', 'foreArm_joint', 'wrist1_joint', 'wrist2_joint', 'wrist3_joint']
                js_msg.position = smooth_joints
                self.pub_state.publish(js_msg)
            else:
                print("[错误] IK无解")
                break
                
            rate.sleep()
            
        print(">>> 到达。")

    def run_console(self):
        print("\n" + "="*50)
        print(">>> 梯形速度规划控制器 (带滤波平滑) <<<")
        print(">>> 特性: 平稳加速 -> 匀速巡航 -> 平稳减速 <<<")
        print("="*50)

        try:
            while not rospy.is_shutdown():
                curr = self.get_current_pose()
                print("\n当前: X={:.4f}, Y={:.4f}, Z={:.4f}".format(curr.p[0], curr.p[1], curr.p[2]))
                print("输入目标 (x y z) 或 'q' 退出:")
                
                user_input = input(">> ")
                if user_input.strip() == 'q': break
                
                try:
                    parts = user_input.split()
                    if len(parts) == 3:
                        tx, ty, tz = map(float, parts)
                        if abs(tx) < 1.2 and abs(ty) < 1.2 and abs(tz) < 1.2:
                            self.move_linear_trap(tx, ty, tz)
                        else:
                            print("坐标超出安全范围!")
                    else:
                        print("格式错误。")
                except ValueError:
                    print("输入无效。")
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.driver.disconnect()

if __name__ == "__main__":
    controller = CartesianController()
    controller.run_console()