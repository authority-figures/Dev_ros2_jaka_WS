import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from my_custom_msgs.msg import RobotStatus
import math


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(RobotStatus, '/robot_arm/state', 10)
        # self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz publishing rate
        self.time = 0  # 用于生成正弦波信号的时间变量

        msg = RobotStatus()
        current_time = self.get_clock().now()
        msg.joint_state.header.stamp = current_time.to_msg()
        print(msg.joint_state.header.stamp)


def main(args=None):
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()

    rclpy.spin(joint_state_publisher)

    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()