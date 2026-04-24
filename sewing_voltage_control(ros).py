#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, String
import minimalmodbus
import serial

class SewingMachineDriverPro:
    def __init__(self):
        rospy.init_node('sewing_machine_driver', anonymous=False)

        # ==========================================
        # 1. 硬件核心参数 (基于万用表实测数据)
        # ==========================================
        self.PORT = rospy.get_param('~port', '/dev/ttyCH341USB0')
        self.SLAVE_ADDRESS = 1
        self.REGISTER_ADDR = 10  # 000AH
        
        # 离散动作电压阈值
        self.V_CUT = 0.37    # 剪线 (全后踏实测)
        self.V_LIFT = 1.45   # 抬压脚 (半后踏实测)
        self.V_STOP = 1.73   # 绝对回中静止 (物理零位实测)

        # 连续调速电压阈值
        self.V_START = 2.22       # 起缝边缘 (越过死区)
        self.V_ACCEL_START = 2.70 # 结束低速，开始纯线性加速点
        self.V_MAX = 4.40         # 满速极值 (满前踩实测)

        # 状态锁：用于防止在执行剪线等需要延时的动作时被速度指令打断
        self.action_lock = False

        # 初始化 Modbus
        self.init_modbus()

        # ==========================================
        # 2. ROS 话题订阅
        # ==========================================
        rospy.Subscriber('/sewing_machine/speed_cmd', Float64, self.speed_callback, queue_size=1)
        rospy.Subscriber('/sewing_machine/action_cmd', String, self.action_callback, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)
        
        rospy.loginfo("缝纫机 Pro 级驱动节点已启动！实测电压映射已加载。")
        self.set_voltage(self.V_STOP)

    def init_modbus(self):
        try:
            self.instrument = minimalmodbus.Instrument(self.PORT, self.SLAVE_ADDRESS)
            self.instrument.serial.baudrate = 9600
            self.instrument.serial.bytesize = 8
            self.instrument.serial.stopbits = 1
            self.instrument.serial.parity = serial.PARITY_NONE
            self.instrument.serial.timeout = 0.1 # 降低超时时间，提高ROS响应频率
            self.instrument.mode = minimalmodbus.MODE_RTU
            rospy.loginfo(f"成功连接至 RS485 模块: {self.PORT}")
        except Exception as e:
            rospy.logerr(f"串口初始化失败！请确认已给予权限: sudo chmod 666 {self.PORT}")
            rospy.logerr(f"详细错误: {e}")
            exit(1)

    def set_voltage(self, voltage):
        """底层写入寄存器的方法"""
        # 硬件安全限幅保护
        if voltage < 0.0: voltage = 0.0
        if voltage > 5.0: voltage = 5.0
        
        register_value = int(voltage * 1000)
        
        try:
            self.instrument.write_register(self.REGISTER_ADDR, register_value, functioncode=6)
            # 使用 debug 级别，避免在高速通信时刷屏
            rospy.logdebug(f"输出电压: {voltage:.2f}V (寄存器: {register_value})")
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Modbus 写入失败: {e}")

    # ==========================================
    # 3. 分段式速度控制逻辑 (核心优化)
    # ==========================================
    def speed_callback(self, msg):
        if self.action_lock:
            return  # 如果正在执行剪线等离散动作，忽略速度指令
            
        speed_percent = msg.data
        
        # 安全限幅 (0.0 ~ 1.0)
        if speed_percent <= 0.0:
            self.set_voltage(self.V_STOP)
            return
        if speed_percent > 1.0:
            speed_percent = 1.0
            
        # 分段映射算法
        if speed_percent <= 0.1:
            # 1% ~ 10%: 映射到低速匀速区 (2.22V ~ 2.70V)
            # 将 0~0.1 归一化为 0~1
            normalized_low = speed_percent / 0.1 
            target_voltage = self.V_START + normalized_low * (self.V_ACCEL_START - self.V_START)
        else:
            # 11% ~ 100%: 映射到高速线性加速区 (2.70V ~ 4.40V)
            # 将 0.1~1.0 归一化为 0~1
            normalized_high = (speed_percent - 0.1) / 0.9 
            target_voltage = self.V_ACCEL_START + normalized_high * (self.V_MAX - self.V_ACCEL_START)
            
        self.set_voltage(target_voltage)

    # ==========================================
    # 4. 离散动作控制逻辑
    # ==========================================
    def action_callback(self, msg):
        action = msg.data.lower()
        
        if action == "stop":
            rospy.loginfo("动作: 紧急回中刹车")
            self.action_lock = False
            self.set_voltage(self.V_STOP)
            
        elif action == "lift":
            rospy.loginfo("动作: 抬起压脚")
            self.action_lock = True
            self.set_voltage(self.V_LIFT)
            
        elif action == "cut":
            rospy.loginfo("动作: 执行自动剪线")
            self.action_lock = True
            self.set_voltage(self.V_CUT)
            
            # 给电控系统预留 0.5 秒时间完成机械剪线动作
            rospy.sleep(0.5) 
            
            rospy.loginfo("剪线完成，自动回中待机")
            self.set_voltage(self.V_STOP)
            self.action_lock = False
            
        else:
            rospy.logwarn(f"未知动作指令: {action}")

    def shutdown_hook(self):
        """节点关闭时的最高优先级安全保护机制"""
        rospy.logwarn("ROS节点关闭，下发物理死区电压 1.73V 以保护缝纫机！")
        self.set_voltage(self.V_STOP)

if __name__ == '__main__':
    try:
        SewingMachineDriverPro()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
