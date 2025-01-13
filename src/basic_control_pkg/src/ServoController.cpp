#include "ServoController.h"

// ServoController 类构造函数
ServoController::ServoController(JAKAZuRobot* robot, JakaController* controller)
    : robot_(robot), controller_(controller)
    //, servo_running(false), standard_step_time(0.008)
{}

errno_t ServoController::servo_enable() {
    errno_t ret;
    ret = robot_->servo_move_enable(TRUE);
    return ret;
}

errno_t ServoController::move_js_extend(const std::vector<JointValue> joints,MoveMode mode = MoveMode::ABS, unsigned int step_num = 1) {
    for 
    return 0;
}