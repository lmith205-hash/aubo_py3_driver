#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

class JointStateCalculator:
    def __init__(self):
        rospy.init_node('joint_state_derivator', anonymous=True)

        # 订阅原始话题
        self.sub = rospy.Subscriber("/joint_states", JointState, self.callback)
        
        # 发布处理后的话题
        self.pub = rospy.Publisher("/joint_states_processed", JointState, queue_size=10)

        self.prev_msg = None
        self.prev_velocities = {} # 存储上一次计算的速度，用于算加速度
        
        # --- 滤波参数设置 ---
        # 0.0 = 不滤波 (原本的噪声)
        # 0.5 = 中等平滑
        # 0.9 = 强力平滑 (会有些许延迟，但曲线很顺滑)
        self.smoothing_alpha = 0.95 
        self.filtered_vels = {} # 存储滤波后的速度状态

        rospy.loginfo("Joint State Derivator Started. Publishing to /joint_states_processed")

    def callback(self, msg):
        # 如果是第一帧数据，只记录不计算
        if self.prev_msg is None:
            self.prev_msg = msg
            # 初始化滤波器状态
            for name in msg.name:
                self.filtered_vels[name] = 0.0
                self.prev_velocities[name] = 0.0
            return

        # 1. 计算时间差 dt
        curr_time = msg.header.stamp.to_sec()
        prev_time = self.prev_msg.header.stamp.to_sec()
        dt = curr_time - prev_time

        # 防止除以0或时间倒流
        if dt <= 0.00001:
            return

        # 准备新的消息
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = msg.name
        new_msg.position = msg.position
        new_msg.velocity = []
        new_msg.effort = [] # 这里我们将用来存放 加速度

        # 将当前消息转为字典方便查找 {name: position}
        curr_pos_map = dict(zip(msg.name, msg.position))
        prev_pos_map = dict(zip(self.prev_msg.name, self.prev_msg.position))

        for name in msg.name:
            # 获取位置
            p_curr = curr_pos_map.get(name, 0.0)
            p_prev = prev_pos_map.get(name, 0.0)

            # --- A. 计算原始速度 ---
            raw_vel = (p_curr - p_prev) / dt

            # --- B. 速度滤波 (Low Pass Filter) ---
            # v_smooth = alpha * v_prev + (1 - alpha) * v_raw
            # 注意：低通滤波公式有很多变种，这里用简单的指数加权移动平均
            prev_smooth_vel = self.filtered_vels.get(name, 0.0)
            smooth_vel = (self.smoothing_alpha * prev_smooth_vel) + ((1.0 - self.smoothing_alpha) * raw_vel)
            
            # 更新滤波器状态
            self.filtered_vels[name] = smooth_vel

            # --- C. 计算加速度 (根据平滑后的速度) ---
            v_prev = self.prev_velocities.get(name, 0.0)
            accel = (smooth_vel - v_prev) / dt
            self.prev_velocities[name] = smooth_vel # 更新用于下一次计算加速度的速度值

            # 填充数据
            new_msg.velocity.append(smooth_vel)
            new_msg.effort.append(accel) # Hack: 用 effort 字段存加速度

        # 发布消息
        self.pub.publish(new_msg)

        # 更新上一帧消息
        self.prev_msg = msg

if __name__ == '__main__':
    try:
        node = JointStateCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass