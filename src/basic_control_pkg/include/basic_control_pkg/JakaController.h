#pragma once
#ifndef JAKACONTROLLER_H
#define JAKACONTROLLER_H
#define _USE_MATH_DEFINES
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <map>
#include <variant>
#include "JAKAZuRobot.h" // 假设 JAKAZuRobot 的头文件路径
#include "ServoController.h"


class ServoController;

class JakaController {
public:
    explicit JakaController(const std::string& ip);
    ~JakaController();
    errno_t login();
    errno_t power_on();
    errno_t enable_robot();
    errno_t logout();
    errno_t joint_move(const JointValue* joint_pos, MoveMode move_mode, BOOL is_block, double
        speed);
    errno_t get_motion_status(MotionStatus* status);

    errno_t get_robot_status(RobotStatus* status);
    errno_t get_joint_position(JointValue* joint_position);

public:
    JAKAZuRobot robot;

private:
    std::string ip_;
    
    int max_joint_speed;
    int max_joint_acc;
    bool servo_turn;
    bool get_robot_info_run;
    std::mutex data_mutex;

    ServoController* servo_controller; // 使用指针而不是静态对象

    // 工具函数
    bool validate_joint_values(const JointValue* joint_pos); 

};






#endif // JAKACONTROLLER_H