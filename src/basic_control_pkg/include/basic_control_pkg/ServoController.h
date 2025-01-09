#pragma once
#include "JAKAZuRobot.h" // 假设 JAKAZuRobot 的头文件路径
#include "JakaController.h"


class JakaController;

class ServoController {
public:
    ServoController(JAKAZuRobot* robot, JakaController* controller);
    errno_t servo_enable();
    errno_t move_j_extend();
    //void servo_move(std::vector<std::vector<double>> joints, double sleep_time = 0);


private:
    JAKAZuRobot* robot_;
    JakaController* controller_;
    bool servo_running;
    double standard_step_time;
};