#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import math
import threading
import random

# Python 2 建议显式继承 object
class AuboRobot(object):
    def __init__(self):
        print "[Sim] AuboRobot 实例已创建 (仿真模式)"
        self.connected = False
        # 初始关节角度 (Aubo i12 的零位)
        self.current_joints = [0.0, 0.0, 1.57, 0.0, 1.57, 0.0]
        self.servo_enabled = False
        
        # 模拟回调相关
        self.speed_callback = None
        self.callback_thread = None
        self.stop_callback = False

    def login(self, ip, port):
        print "[Sim] 正在连接虚拟控制器 IP: %s Port: %d ..." % (ip, port)
        time.sleep(0.5) # 模拟网络延迟
        self.connected = True
        print "[Sim] 登录成功！"
        return True

    def logout(self):
        print "[Sim] 断开连接。"
        self.stop_callback = True
        if self.callback_thread:
            self.callback_thread.join()
        self.connected = False

    def get_current_joints(self):
        if not self.connected:
            raise Exception("Not connected")
        return list(self.current_joints)

    def enable_servo(self):
        if not self.connected:
            return False
        print "[Sim] 透传模式已开启"
        self.servo_enabled = True
        return True

    def disable_servo(self):
        print "[Sim] 透传模式已关闭"
        self.servo_enabled = False
        return True

    def send_joints(self, joints):
        if not self.connected or not self.servo_enabled:
            return
        if len(joints) != 6:
            print "[Sim] 错误：关节数量不对"
            return
        # 简单赋值
        self.current_joints = list(joints)

    def follow_joint(self, joints):
        if not self.connected:
            return -1
        self.current_joints = list(joints)
        return 0

    # ==============================
    # 新增：仿真速度回调
    # ==============================
    def _sim_speed_loop(self):
        """后台线程模拟SDK推送速度数据"""
        t = 0
        while not self.stop_callback:
            if self.speed_callback:
                # 模拟一个正弦波速度 + 随机噪声 (单位 m/s)
                sim_speed = 0.5 * math.sin(t) + random.uniform(-0.05, 0.05)
                try:
                    self.speed_callback(sim_speed)
                except Exception as e:
                    print "[Sim Error] Callback failed: %s" % str(e)
            time.sleep(0.05) # 20Hz 推送
            t += 0.1

    def register_speed_callback(self, callback_func):
        """
        仿真注册函数
        """
        print "[Sim] 注册末端速度回调函数..."
        self.speed_callback = callback_func
        
        # 启动仿真线程
        if self.callback_thread is None:
            self.stop_callback = False
            self.callback_thread = threading.Thread(target=self._sim_speed_loop)
            self.callback_thread.daemon = True
            self.callback_thread.start()
        
        return True