#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "JakaController.h"
#include "my_custom_msgs/msg/robot_status.hpp"



class RobotArmController : public rclcpp::Node {
public:
    RobotArmController() : Node("robot_arm_controller") {
        // 初始化机械臂控制器
        controller_ = std::make_unique<JakaController>("10.5.5.100");
        if (controller_->login() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to login to robot.");
            rclcpp::shutdown();
        }
        controller_->power_on();
        controller_->enable_robot();

        // 初始化订阅器
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/robot_arm/joint_commands",
            10,
            std::bind(&RobotArmController::joint_command_callback, this, std::placeholders::_1)
        );

        // 初始化发布器
        publisher_ = this->create_publisher<my_custom_msgs::msg::RobotStatus>("/robot_arm/state", 10);

        // 创建状态监控线程
        monitoring_thread_ = std::thread(&RobotArmController::monitor_robot_state, this);

        RCLCPP_INFO(this->get_logger(), "RobotArmController node initialized.");
    }

    ~RobotArmController() {
        if (monitoring_thread_.joinable()) {
            stop_monitoring_ = true;
            monitoring_thread_.join();
        }
        controller_->logout();
    }

private:
    // 回调函数：处理关节控制命令
    void joint_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint command. Expected 6 values.");
            return;
        }

        // 构造 JointValue 对象
        JointValue joint_pos;
        for (size_t i = 0; i < 6; ++i) {
            joint_pos.jVal[i] = msg->data[i];
        }

        // 调用 joint_move
        errno_t ret = controller_->joint_move(&joint_pos, MoveMode::ABS, true, 5.0);
        if (ret == 0) {
            RCLCPP_INFO(this->get_logger(), "Joint command executed successfully.");
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute joint command. Error code: %d", ret);
        }
    }

    // 线程函数：监控机械臂状态
    void monitor_robot_state() {
        while (!stop_monitoring_ && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 获取机械臂状态（模拟获取状态，需根据实际 API 实现）
            RobotStatus robot_status;
            // JointValue current_joint_pos;
            // errno_t ret = controller_->robot.get_joint_position(&current_joint_pos);
            errno_t ret = controller_->get_robot_status(&robot_status);


            if (ret == 0) {
                // 构造并发布 JointState 消息
                // auto msg = sensor_msgs::msg::JointState();
                auto msg = my_custom_msgs::msg::RobotStatus();
                
                msg.joint_state.header.stamp = this->now();
                msg.joint_state.name = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
                msg.joint_state.position = { robot_status.joint_position[0], robot_status.joint_position[1],
                                robot_status.joint_position[2], robot_status.joint_position[3],
                                robot_status.joint_position[4], robot_status.joint_position[5] };
                msg.joint_state.velocity = {
                    robot_status.robot_monitor_data.jointMonitorData[0].instVel,robot_status.robot_monitor_data.jointMonitorData[1].instVel,
                    robot_status.robot_monitor_data.jointMonitorData[2].instVel,robot_status.robot_monitor_data.jointMonitorData[3].instVel,
                    robot_status.robot_monitor_data.jointMonitorData[4].instVel,robot_status.robot_monitor_data.jointMonitorData[5].instVel
                };
                msg.joint_state.effort = {
                    robot_status.robot_monitor_data.jointMonitorData[0].instTorq,robot_status.robot_monitor_data.jointMonitorData[1].instTorq,
                    robot_status.robot_monitor_data.jointMonitorData[2].instTorq,robot_status.robot_monitor_data.jointMonitorData[3].instTorq,
                    robot_status.robot_monitor_data.jointMonitorData[4].instTorq,robot_status.robot_monitor_data.jointMonitorData[5].instTorq
                };
                msg.force_torque= {
                    0,1,2,3,4,5
                };
            
                publisher_->publish(msg);
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Failed to get joint position. Error code: %d", ret);
            }
        }
    }

    std::unique_ptr<JakaController> controller_; // 机械臂控制器
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_; // 订阅器
    rclcpp::Publisher<my_custom_msgs::msg::RobotStatus>::SharedPtr publisher_; // 发布器
    std::thread monitoring_thread_; // 状态监测线程
    std::atomic<bool> stop_monitoring_{ false }; // 监控线程停止标志
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
