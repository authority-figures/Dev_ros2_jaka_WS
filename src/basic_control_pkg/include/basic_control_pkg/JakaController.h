#ifndef JAKACONTROLLER_H
#define JAKACONTROLLER_H

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

class JakaController {
public:
    explicit JakaController(const std::string& ip);
    ~JakaController();
    errno_t login();
    void power_on();
    void enable_robot();
    void logout();

private:
    std::string ip_;
    JAKAZuRobot* robot;
    int max_joint_speed;
    int max_joint_acc;
    bool servo_turn;
    bool get_robot_info_run;
    std::mutex data_mutex;





    class ServoController {
    public:
        ServoController(JAKAZuRobot* robot, JakaController* controller);
        errno_t servo_enable();
        errno_t move_j_extend();
        void servo_move(std::vector<std::vector<double>> joints, double sleep_time = 0);
        

    private:
        JAKAZuRobot* robot_;
        JakaController* controller_;
        bool servo_running;
        double standard_step_time;
    };

    ServoController* servo_controller;

};






#endif // JAKACONTROLLER_H