#include "JakaController.h"

// ���캯������������
JakaController::JakaController(const std::string& ip)
    : ip_(ip), max_joint_speed(180), max_joint_acc(90), servo_turn(false), get_robot_info_run(false) {
    robot = new JAKAZuRobot();
}

JakaController::~JakaController() {
    delete robot;
}

// ��¼
errno_t JakaController::login() {
    errno_t ret = robot->login_in(ip_.c_str()); // ���õײ� SDK �ĵ�¼����

    if (ret == ERR_SUCC) { // �����¼�ɹ�
        servo_controller = new ServoController(robot, this);
        std::cout << "Login successful!" << std::endl;
    }
    else
    {
        std::cerr << "Login failed with error code: " << ret << std::endl;
    }
    
    return ret; // ���� SDK �����Ĵ������ɹ�״̬
}



// ������Դ
void JakaController::power_on() {
    robot->power_on();
}

// ʹ�ܻ�����
void JakaController::enable_robot() {
    robot->enable_robot();
}

// �ǳ�
void JakaController::logout() {
    robot->disable_robot();
    robot->login_out();
}

// ServoController �๹�캯��
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