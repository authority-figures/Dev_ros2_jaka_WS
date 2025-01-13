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
    for (size_t i = 0; i < joints.size(); ++i) {
        JointValue joint_pos = joints[i];
        if (!controller_->validate_joint_values(&joint_pos)) {
            std::cerr << "move_js_extend failed: joint values out of range!" << std::endl;
            return ERR_INVALID_PARAMETER;
        }

        // 调用 joint_move 指令
        errno_t ret = robot_->servo_j(&joint_pos, mode, step_num);
        if (ret != ERR_SUCC) {
            std::cerr << "move_js_extend failed: error code " << ret << std::endl;
            return ret;
        }

    }
    return ERR_SUCC;
    
}