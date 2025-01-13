import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from my_custom_msgs.msg import Servoj, JointArray # 根据你的消息定义修改 import 路径
import time

class TestServoControlNode(Node):
    def __init__(self):
        super().__init__('test_servo_control_node')
        
        # 发布 Servoj 和 JointArray 的话题
        self.servoj_publisher = self.create_publisher(Servoj, '/robot_arm/servo_control', 10)
        self.joint_array_publisher = self.create_publisher(JointArray, '/robot_arm/servo_control_joints', 10)
        self.enable_publisher = self.create_publisher(String, '/robot_arm/servo_enable', 10)
    def pubish_enable(self):
        msg = String()
        msg.data = "enable"
        self.enable_publisher.publish(msg)
    def publish_servoj(self):
        # 创建 Servoj 消息
        msg = Servoj()
        msg.joint_state.position = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]  # 示例关节位置
        msg.movemode = 1  # 假设 1 表示某种运动模式
        msg.step_num = 1  # 示例步长
        
        # 发布 Servoj 消息
        self.servoj_publisher.publish(msg)
        self.get_logger().info(f'Published Servoj message: {msg}')

    def publish_joint_array(self):
        # 创建 JointArray 消息
        msg = JointArray()
        
        # 添加第一个 Servoj 子消息
        joint1 = JointState()
        joint1.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        msg.joint_array.append(joint1)
        
        # 添加第二个 Servoj 子消息
        joint2 = JointState()
        joint2.position = [1.0, 1.1, 1.2, 1.3, 1.4, 1.5]
        msg.joint_array.append(joint2)

        msg.movemode = 1  # 假设 2 表示另一种运动模式
        msg.step_num = 2
        # 发布 JointArray 消息
        self.joint_array_publisher.publish(msg)
        self.get_logger().info(f'Published JointArray message: {msg}')


def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = TestServoControlNode()
    
    try:
        node.get_logger().info("Starting test_servo_control_node...")
        node.pubish_enable()
        time.sleep(5)  # 等待 1 秒

        # 发布 Servoj 消息
        node.get_logger().info("Publishing Servoj message...")
        node.publish_servoj()
        time.sleep(5)  # 等待 1 秒
        
        # 发布 JointArray 消息
        node.get_logger().info("Publishing JointArray message...")
        node.publish_joint_array()
        time.sleep(5)  # 等待 1 秒

    except KeyboardInterrupt:
        pass

    # 关闭节点
    node.get_logger().info("Shutting down node...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
