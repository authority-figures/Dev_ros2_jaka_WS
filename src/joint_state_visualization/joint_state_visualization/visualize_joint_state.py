import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from my_custom_msgs.msg import RobotStatus
import matplotlib.pyplot as plt
import numpy as np

class JointStateVisualizer(Node):
    def __init__(self):
        super().__init__('joint_state_visualizer')
        # msg1 = RobotStatus()

        # 订阅 /robot_arm/state 话题，获取 JointState 数据
        self.subscription = self.create_subscription(
            JointState,
            '/robot_arm/state',
            self.joint_state_callback,
            10
        )

        # 用于存储关节位置数据
        self.joint_positions = np.zeros(6)

        # 初始化绘图
        self.figure, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.joint_positions, label='Joint Position')
        self.ax.set_ylim(-3.0, 3.0)  # 根据需要设置 Y 轴范围
        self.ax.set_xlabel('Joint Index')
        self.ax.set_ylabel('Position (rad)')
        self.ax.legend()
        plt.ion()  # 打开交互模式
        plt.show()

    def joint_state_callback(self, msg):
        # 获取关节位置数据并更新
        self.joint_positions = np.array(msg.position)

        # 更新绘图
        self.line.set_ydata(self.joint_positions)
        self.ax.relim()  # 调整轴范围
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.1)  # 控制更新频率，防止过于频繁的更新

def main(args=None):
    rclpy.init(args=args)
    node = JointStateVisualizer()
    
    # ROS 2 运行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
