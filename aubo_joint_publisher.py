#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_matrix

try:
    import PyKDL
    from kdl_parser_py.urdf import treeFromFile
except ImportError:
    PyKDL = None
    treeFromFile = None

# 导入我们已经加固过的统一驱动
from unified_driver import AuboDriver

# ================= 配置区域 =================
REAL_IP = "100.100.1.10"
PUBLISH_RATE = 100.0  # 发布频率 (Hz)，手眼标定采集数据 50Hz 足够了
URDF_PATH = "/home/lmy/aubo_i12/src/aubo_robot/aubo_robot/aubo_description/urdf/aubo_i12.urdf"
BASE_LINK = "base_link"
TIP_LINK = "wrist3_Link"
# ============================================


class RobotFkSolver(object):
    def __init__(self, urdf_path, base_link, tip_link, joint_names):
        if PyKDL is None or treeFromFile is None:
            raise RuntimeError("缺少 PyKDL 或 kdl_parser_py，无法计算法兰位姿。")

        success, tree = treeFromFile(urdf_path)
        if not success:
            raise RuntimeError("KDL 解析 URDF 失败: {}".format(urdf_path))

        self.chain = tree.getChain(base_link, tip_link)
        self.num_joints = self.chain.getNrOfJoints()
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
        self.joint_names = joint_names

        if self.num_joints != len(self.joint_names):
            rospy.logwarn(
                "URDF 链关节数为 %d，但当前 joint_names 为 %d，默认按前 %d 个使用。",
                self.num_joints,
                len(self.joint_names),
                self.num_joints,
            )

    def compute_pose(self, joint_positions):
        q = PyKDL.JntArray(self.num_joints)
        for idx in range(self.num_joints):
            q[idx] = joint_positions[idx]

        frame = PyKDL.Frame()
        ret = self.fk_solver.JntToCart(q, frame)
        if ret < 0:
            raise RuntimeError("KDL FK 求解失败，返回值: {}".format(ret))

        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = np.array(
            [
                [frame.M[0, 0], frame.M[0, 1], frame.M[0, 2]],
                [frame.M[1, 0], frame.M[1, 1], frame.M[1, 2]],
                [frame.M[2, 0], frame.M[2, 1], frame.M[2, 2]],
            ],
            dtype=np.float64,
        )
        transform[:3, 3] = np.array([frame.p[0], frame.p[1], frame.p[2]], dtype=np.float64)
        return transform


def transform_to_pose_msg(transform, stamp, frame_id):
    pose_msg = PoseStamped()
    pose_msg.header.stamp = stamp
    pose_msg.header.frame_id = frame_id
    quat = quaternion_from_matrix(transform)
    pose_msg.pose.position.x = float(transform[0, 3])
    pose_msg.pose.position.y = float(transform[1, 3])
    pose_msg.pose.position.z = float(transform[2, 3])
    pose_msg.pose.orientation.x = float(quat[0])
    pose_msg.pose.orientation.y = float(quat[1])
    pose_msg.pose.orientation.z = float(quat[2])
    pose_msg.pose.orientation.w = float(quat[3])
    return pose_msg


class AuboJointPublisher(object):
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('aubo_joint_publisher', anonymous=True)

        # 1. 实例化驱动 (关闭仿真，使用真机IP)
        self.driver = AuboDriver(use_simulation=False, real_ip=REAL_IP)

        # 标准 Aubo i12 的关节名称，需与您的 URDF 文件中的名字严格匹配
        self.joint_names = [
            'shoulder_joint',
            'upperArm_joint',
            'foreArm_joint',
            'wrist1_joint',
            'wrist2_joint',
            'wrist3_joint'
        ]

        # 2. 初始化 FK 求解器
        if not os.path.exists(URDF_PATH):
            rospy.logerr("找不到 URDF 文件: %s", URDF_PATH)
            sys.exit(1)
        try:
            self.fk_solver = RobotFkSolver(URDF_PATH, BASE_LINK, TIP_LINK, self.joint_names)
        except Exception as exc:
            rospy.logerr("FK 求解器初始化失败: %s", exc)
            sys.exit(1)

        # 3. 初始化发布者
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.pose_pub = rospy.Publisher('/aubo_pose', PoseStamped, queue_size=10)

    def run(self):
        # 4. 建立连接
        if not self.driver.connect():
            rospy.logerr("连接机械臂失败，程序退出。")
            sys.exit(1)

        # 【重点说明】：这里绝对不要调用 self.driver.enable_servo()！
        # 只要不开启透传模式，机械臂的控制权就一直在示教器手里，
        # 而 SDK 依然可以在后台静默读取关节数据。

        rospy.loginfo("================================================")
        rospy.loginfo("机械臂连接成功！进入【只读监听模式】。")
        rospy.loginfo("您可以直接使用示教器自由移动机械臂。")
        rospy.loginfo("正在发布 /joint_states 和 /aubo_pose ... (%.1f Hz)", PUBLISH_RATE)
        rospy.loginfo("================================================")

        rate = rospy.Rate(PUBLISH_RATE)

        try:
            while not rospy.is_shutdown():
                # 从底层 SDK 获取当前 6 个关节的角度 (弧度)
                joints = self.driver.get_current_joints()

                # 检查数据有效性
                if joints and len(joints) == 6:
                    stamp = rospy.Time.now()

                    # 发布关节角
                    js_msg = JointState()
                    js_msg.header.stamp = stamp
                    js_msg.header.frame_id = BASE_LINK
                    js_msg.name = self.joint_names
                    js_msg.position = joints
                    self.joint_pub.publish(js_msg)

                    # 发布法兰位姿
                    try:
                        base_flange = self.fk_solver.compute_pose(joints)
                        pose_msg = transform_to_pose_msg(base_flange, stamp, BASE_LINK)
                        self.pose_pub.publish(pose_msg)
                    except Exception as exc:
                        rospy.logerr_throttle(2.0, "FK 计算失败: %s", exc)
                else:
                    rospy.logwarn_throttle(2.0, "获取关节数据失败或不完整。")

                rate.sleep()

        except Exception as e:
            rospy.logerr("运行过程发生异常: {}".format(e))
        finally:
            # 退出时依靠 unified_driver.py 内部的机制安全断开
            pass


if __name__ == "__main__":
    try:
        publisher = AuboJointPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
