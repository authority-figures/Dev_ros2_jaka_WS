#include "JakaController.h"

// 构造函数和析构函数
JakaController::JakaController(const std::string& ip)
    : ip_(ip), max_joint_speed(180), max_joint_acc(90), servo_turn(false), get_robot_info_run(false) {
    robot = new JAKAZuRobot();
}

JakaController::~JakaController() {
    delete robot;
}

// 登录
errno_t JakaController::login() {
    errno_t ret = robot->login_in(ip_.c_str()); // 调用底层 SDK 的登录函数

    if (ret == ERR_SUCC) { // 如果登录成功
        servo_controller = new ServoController(robot, this);
        std::cout << "Login successful!" << std::endl;
    }
    else
    {
        std::cerr << "Login failed with error code: " << ret << std::endl;
    }
    
    return ret; // 返回 SDK 函数的错误码或成功状态
}



// 启动电源
void JakaController::power_on() {
    robot->power_on();
}

// 使能机器人
void JakaController::enable_robot() {
    robot->enable_robot();
}

// 登出
void JakaController::logout() {
    robot->disable_robot();
    robot->login_out();
}

// ServoController 类构造函数
JakaController::ServoController::ServoController(JAKAZuRobot* robot, JakaController* controller)
    : robot_(robot), controller_(controller), servo_running(false), standard_step_time(0.008) {}

errno_t JakaController::ServoController::servo_enable() {
    errno_t ret;
    ret = robot_->servo_move_enable(TRUE);
    return ret;
}

errno_t JakaController::ServoController::move_j_extend() {
    // if controller_->
    return 0;
}