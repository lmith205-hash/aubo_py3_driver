#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import math
import rospy
from sensor_msgs.msg import JointState

# 直接导入，不使用 as 别名，避免命名空间混淆
try:
    import aubo_py3
except ImportError as e:
    print("[Error] 无法导入 aubo_py3: {}".format(e))
    print("请确保编译好的 aubo_py3.so 文件存在于当前目录，或者在 PYTHONPATH 中！")
    sys.exit(1)

class AuboDriver(object):
    def __init__(self, use_simulation=False, real_ip='192.168.1.10'):
        self.use_simulation = use_simulation
        self.robot_ip = real_ip
        self.connected = False
        self.robot = None
        
        # 缓存数据
        self.current_joints = [0.0] * 6
        
        # 仿真模式下的虚拟状态
        if self.use_simulation:
            self.sim_joints = [1.57, 0.0, 1.57, 0.0, 1.57, 0.0]

        # 【核心加固 1】：注册底层关闭钩子，确保 Ctrl+C 一定会触发断开序列
        rospy.on_shutdown(self._ros_shutdown_hook)

    def _ros_shutdown_hook(self):
        """捕获 Ctrl+C 的紧急回调"""
        if self.connected:
            print("\n[System] 捕获到程序退出信号(Ctrl+C)，启动紧急断开保护机制...")
            self.disconnect()

    def connect(self):
        """连接机械臂"""
        if self.use_simulation:
            self.connected = True
            rospy.loginfo("[Sim] Aubo driver connected (Simulation)")
            return True
            
        if self.connected:
            print("Aubo driver is already connected")
            return True
            
        try:
            # 现有的 C++ 库暴露的名字依然是 AuboRobot
            self.robot = aubo_py3.AuboRobot() 
            
            # C++ 中的 login 函数接收 2 个参数 (ip, port)
            success = self.robot.login(self.robot_ip, 8899)
            
            if success: 
                self.connected = True
                rospy.loginfo("Aubo robot connected: {}".format(self.robot_ip))
                
                # 连接成功后获取一次初始状态
                self.get_current_joints()
                return True
            else:
                rospy.logerr("Login failed (Check IP or Controller State)")
                return False
                
        except Exception as e:
            rospy.logerr("Exception during connection: {}".format(e))
            print("Failed to connect to Aubo robot: {}".format(self.robot_ip))
            return False

    def disconnect(self):
        """断开连接"""
        if self.use_simulation:
            self.connected = False
            return
            
        if self.connected and self.robot:
            print("\n" + "="*50)
            print(">>> 开始交还机械臂控制权 <<<")
            try:
                # 【终极安全加固 2】：多帧平滑刹车 (Soft Stop)
                # 机械臂抖动是因为透传指令突然中断，缓冲区瞬间清空。
                # 解决方法：连续发送同一“驻留坐标”填补缓冲区，强迫速度平滑降至绝对0
                if hasattr(self, 'current_joints') and self.current_joints:
                    print("[System] 正在执行平滑刹车序列(0.5秒)...")
                    # 重新获取一次当前最准确的物理位置
                    stop_joints = self.robot.get_current_joints()
                    if stop_joints and len(stop_joints) == 6:
                        # 连续发送 50 次，每次间隔 10ms (相当于 100Hz 频率维持 0.5 秒)
                        for _ in range(50):
                            self.robot.send_joints(stop_joints)
                            time.sleep(0.01)

                # 真实调用 C++ 的透传退出接口
                success = self.robot.disable_servo()
                
                # 【核心加固 3】：此时禁用 rospy.loginfo，只用 print
                if success:
                    print("[System] ✅ 成功退出伺服透传模式！")
                else:
                    print("[System] ❌ 退出伺服透传模式失败。")
                
                # 强行等待，让底层硬件状态机彻底切换完毕
                time.sleep(0.5) 
                print("[System] 控制权已交还示教器。")
            except Exception as e:
                print("[Error] 退出序列发生异常: {}".format(e))
                
            self.robot.logout()
            self.connected = False
            print(">>> Aubo robot 已安全断开连接 <<<")
            print("="*50 + "\n")

    def enable_servo(self):
        """开启伺服模式 (透传)"""
        if self.use_simulation: return True
        if not self.connected: return False
        return self.robot.enable_servo()

    def disable_servo(self):
        """关闭伺服模式"""
        if self.use_simulation: return True
        if not self.connected: return False
        return self.robot.disable_servo()

    def get_current_joints(self):
        """获取当前关节角"""
        if self.use_simulation:
            return self.sim_joints
            
        if self.connected:
            self.current_joints = self.robot.get_current_joints()
            return self.current_joints
        return [0.0] * 6

    def send_joints(self, joint_list):
        """发送关节角控制命令"""
        if len(joint_list) != 6:
            return False
            
        if self.use_simulation:
            self.sim_joints = joint_list
            return True
            
        if self.connected:
            return self.robot.send_joints(joint_list)
        return False

    def set_arrival_ahead_time(self, t):
        """设置提前到位时间 (非常适合透传模式)"""
        if self.use_simulation: return True
        if not self.connected: return False
        return self.robot.set_arrival_ahead_time(t)
        
    def set_no_arrival_ahead(self):
        """取消提前到位 (精准到位)"""
        if self.use_simulation: return True
        if not self.connected: return False
        return self.robot.set_no_arrival_ahead()

    def set_io(self, pin, value):
        if self.use_simulation: return
        print("IO configuration is not implemented yet")