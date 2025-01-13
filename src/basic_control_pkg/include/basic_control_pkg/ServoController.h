#pragma once
#include "JAKAZuRobot.h" // ???? JAKAZuRobot ??????��??
#include "JakaController.h"
#include "my_custom_msgs/msg/joint_array.hpp"

class JakaController;

class ServoController {
public:
    ServoController(JAKAZuRobot* robot, JakaController* controller);
    errno_t servo_enable();
    errno_t move_js_extend(const std::vector<JointValue> joints,MoveMode mode , unsigned int step_num );
    //void servo_move(std::vector<std::vector<double>> joints, double sleep_time = 0);


private:
    JAKAZuRobot* robot_;
    JakaController* controller_;
    bool servo_running;
    double standard_step_time;
};