#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from my_custom_msgs.msg import RobotStatus
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, '/robot_arm/state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz publishing rate
        self.time = 0  # 用于生成正弦波信号的时间变量

    def timer_callback(self):
        # 创建消息
        msg = RobotStatus()
        
  

        # 获取当前时间戳
        current_time = self.get_clock().now()
        timestamp_ms = current_time.nanoseconds / 1e6  # 转换为毫秒
        timestamp_s = current_time.nanoseconds / 1e9  # 转换为秒
        msg.joint_state.header.stamp = current_time.to_msg()
        msg.joint_state.header.frame_id = 'base_link'
        
        # 设置关节的名称
        msg.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # 生成六个关节的角度（使用不同相位的正弦波）
        frequency = 1.0  # 频率：1Hz
        amplitude = 1.0  # 振幅：1 rad

        # 六个关节的角度根据不同相位的正弦波生成
        msg.joint_state.position = [
            amplitude * math.sin(timestamp_s + 0),  # joint_1
            amplitude * math.sin(timestamp_s + math.pi / 3),  # joint_2, phase shift by 60 degrees
            amplitude * math.sin(timestamp_s + 2 * math.pi / 3),  # joint_3, phase shift by 120 degrees
            amplitude * math.sin(timestamp_s + math.pi),  # joint_4, phase shift by 180 degrees
            amplitude * math.sin(timestamp_s + 4 * math.pi / 3),  # joint_5, phase shift by 240 degrees
            amplitude * math.sin(timestamp_s + 5 * math.pi / 3)   # joint_6, phase shift by 300 degrees
        ]
        
        # 设置关节的速度：通过求导来近似得到速度（即每个关节角度的变化速.joint_state率）
        # 通过差分计算（简化版），可以更精确地计算出速度
        msg.joint_state.velocity = [
            frequency * amplitude * math.cos(timestamp_s + 0),  # joint_1 velocity
            frequency * amplitude * math.cos(timestamp_s + math.pi / 3),  # joint_2 velocity
            frequency * amplitude * math.cos(timestamp_s + 2 * math.pi / 3),  # joint_3 velocity
            frequency * amplitude * math.cos(timestamp_s + math.pi),  # joint_4 velocity
            frequency * amplitude * math.cos(timestamp_s + 4 * math.pi / 3),  # joint_5 velocity
            frequency * amplitude * math.cos(timestamp_s + 5 * math.pi / 3)   # joint_6 velocit.joint_statey
        ]
        
        # 设置关节的力矩（暂时给一个常数值）
        msg.joint_state.effort = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

        # 增加时间
        self.time += 0.1  # 每次调用时增加时间，模拟时间流逝

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)

    # 销毁节点
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
