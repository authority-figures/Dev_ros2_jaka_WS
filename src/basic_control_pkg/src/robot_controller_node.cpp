
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "JakaController.h"
#include <fmt/core.h>
#include <fmt/ranges.h>
#include "my_custom_msgs/msg/robot_status.hpp"
#include "my_custom_msgs/msg/joint_move.hpp"
#include "my_custom_msgs/msg/servoj.hpp"

class RobotControllerNode : public rclcpp::Node {
public:
    RobotControllerNode()
        : Node("robot_controller_node"),
          controller_("192.168.2.100"), // 替换为实际IP
          executing_task_(false),
          servo_enabled_(false) {

        // 初始化机械臂控制
        if (controller_.login() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to login to robot.");
            rclcpp::shutdown();
        }
        controller_.power_on();
        controller_.enable_robot();

        // 初始化发布器
        publisher_ = this->create_publisher<my_custom_msgs::msg::RobotStatus>("/robot_arm/state", 10);

        // 创建状态监控线程
        monitoring_thread_ = std::thread(&RobotControllerNode::monitor_robot_state, this);

        

        // 订阅伺服控制话题
        servo_control_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_arm/servo_control", 10,
            std::bind(&RobotControllerNode::servo_control_callback, this, std::placeholders::_1));

        // 订阅伺服控制使能话题
        servo_enable_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_arm/servo_enable", 10,
            std::bind(&RobotControllerNode::servo_enable_callback, this, std::placeholders::_1));

        // 订阅关节移动话题
        joint_move_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/robot_arm/joint_move", 10,
            std::bind(&RobotControllerNode::joint_move_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "RobotControllerNode initialized.");
    }

    ~RobotControllerNode() {
        controller_.logout();
    }

private:
    JakaController controller_; // 机械臂控制对象
    std::mutex task_mutex_;     // 任务锁，防止干涉
    std::thread monitoring_thread_; // 状态监测线程
    bool executing_task_;       // 当前是否在执行任务
    bool servo_enabled_;        // 是否启用了伺服控制
    std::atomic<bool> stop_monitoring_{ false }; // 监控线程停止标志

    // 订阅器
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_control_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr servo_enable_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_move_subscription_;

    // 发布器
    rclcpp::Publisher<my_custom_msgs::msg::RobotStatus>::SharedPtr publisher_; 

    // 伺服控制使能回调
    void servo_enable_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (msg->data == "enable") {
            if (executing_task_) {
                RCLCPP_WARN(this->get_logger(), "Cannot enable servo: another task is running.");
                return;
            }
            servo_enabled_ = true;
            errno_t ret = controller_.servo_controller->servo_enable();
            if (ret == ERR_SUCC) {
                RCLCPP_INFO(this->get_logger(), "Servo control enabled.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to enable servo control. Error code: %d", ret);
            }
            RCLCPP_INFO(this->get_logger(), "Servo control enabled.");
        } else if (msg->data == "disable") {
            servo_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "Servo control disabled.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid servo enable command: %s", msg->data.c_str());
        }
    }

    // 伺服控制回调
    void servo_control_callback(const my_custom_msgs::msg::Servoj::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (!servo_enabled_) {
            RCLCPP_WARN(this->get_logger(), "Servo control request ignored: servo control is not enabled.");
            return;
        }
        if (executing_task_) {
            RCLCPP_WARN(this->get_logger(), "Servo control request ignored: another task is running.");
            return;
        }
        executing_task_ = true;
        RCLCPP_INFO(this->get_logger(), "Executing servo control. Positions: %s", fmt::format("{}", fmt::join(msg->joint_state.position, ", ")).c_str());
        JointValue* joint_pos;
        for (size_t i = 0; i < std::min(msg->joint_state.position.size(), 6ul); ++i) {
            joint_pos->jVal[i] = msg->joint_state.position[i];
        }
        // 调用伺服控制指令
        controller_.robot.servo_j(joint_pos, static_cast<MoveMode>(msg->movemode),msg->step_num);
        executing_task_ = false;
    }

    // 关节移动回调
    void joint_move_callback(const my_custom_msgs::msg::JointMove::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (servo_enabled_) {
            RCLCPP_WARN(this->get_logger(), "Joint move request ignored: servo control is enabled.");
            return;
        }
        if (executing_task_) {
            RCLCPP_WARN(this->get_logger(), "Joint move request ignored: another task is running.");
            return;
        }
        executing_task_ = true;
        RCLCPP_INFO(this->get_logger(), "Executing joint move.");

        // 创建关节目标位置
        my_custom_msgs::msg::JointMove* msg;
        
        sensor_msgs::msg::JointState l;
        JointValue joint_pos;

        if (msg->joint_state.position.size() >= 6) {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint command. Expected 6 values.");
            return;
        }

        for (size_t i = 0; i < std::min(msg->joint_state.position.size(), 6ul); ++i) {
            joint_pos.jVal[i] = msg->joint_state.position[i];
        }
        MoveMode move_mode = static_cast<MoveMode>(msg->movemode);
        // 调用 joint_move 指令
        errno_t ret =controller_.joint_move(&joint_pos, move_mode, msg->is_block, msg->speed) == 0;
        if (ret == ERR_SUCC) {
            RCLCPP_INFO(this->get_logger(), "Joint move executed successfully.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute joint command. Error code: %d", ret);
        }
        executing_task_ = false;
    }





    // 线程函数：监控机械臂状态
    void monitor_robot_state() {
        while (!stop_monitoring_ && rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            // 获取机械臂状态（模拟获取状态，需根据实际 API 实现）
            RobotStatus robot_status;
            // JointValue current_joint_pos;
            // errno_t ret = controller_->robot.get_joint_position(&current_joint_pos);
            errno_t ret = controller_.get_robot_status(&robot_status);


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

};
