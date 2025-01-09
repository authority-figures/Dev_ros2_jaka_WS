#include "JakaController.h"

// 构造函数和析构函数
JakaController::JakaController(const std::string& ip)
    : ip_(ip), max_joint_speed(180.0/180*M_PI), max_joint_acc(90 / 180 * M_PI), servo_turn(false), get_robot_info_run(false), robot()
    , servo_controller(nullptr) // 初始化为 nullptr
{
    // 动态分配 ServoController 对象
    servo_controller = new ServoController(&robot, this);
    
}

JakaController::~JakaController() {
    // 删除动态分配的 ServoController 对象
    if (servo_controller) {
        delete servo_controller;
        servo_controller = nullptr;
    }
}


// 定义错误码和错误信息映射表
std::string get_error_message(errno_t error_code) {
    // 使用 std::map 来存储错误码和错误信息
    static const std::map<errno_t, std::string> error_messages = {
        {ERR_SUCC, "Success"},
        {ERR_FUCTION_CALL_ERROR, "Function call error: controller does not support this interface"},
        {ERR_INVALID_HANDLER, "Invalid handler"},
        {ERR_INVALID_PARAMETER, "Invalid parameter"},
        {ERR_COMMUNICATION_ERR, "Communication connection error"},
        {ERR_KINE_INVERSE_ERR, "Kinematic inverse calculation failed"},
        {ERR_EMERGENCY_PRESSED, "Emergency stop button pressed"},
        {ERR_NOT_POWERED, "Robot arm not powered"},
        {ERR_NOT_ENABLED, "Robot arm not enabled"},
        {ERR_DISABLE_SERVOMODE, "Robot arm not in servo mode"},
        {ERR_NOT_OFF_ENABLE, "Robot arm enable is not turned off"},
        {ERR_PROGRAM_IS_RUNNING, "Program is running, operation not allowed"},
        {ERR_CANNOT_OPEN_FILE, "Cannot open file, file does not exist"},
        {ERR_MOTION_ABNORMAL, "Motion abnormality detected"},
        {ERR_FTP_PERFORM	, "FTP operation error"}
    };

    // 查找错误码对应的错误信息
    auto it = error_messages.find(error_code);
    if (it != error_messages.end()) {
        return it->second; // 返回对应的错误信息
    }
    else {
        return "Unknown error"; // 未知错误
    }
}


// 登录
errno_t JakaController::login() {
    errno_t ret = robot.login_in(ip_.c_str()); // 调用底层 SDK 的登录函数

    if (ret == ERR_SUCC) { // 如果登录成功
       
        std::cout << "Login successful!" << std::endl;
    }
    else
    {
        // 打印错误码对应的错误信息
        std::cerr << "Login failed with error code: " << ret
            << " (" << get_error_message(ret) << ")" << std::endl;
    }
    
    return ret; // 返回 SDK 函数的错误码或成功状态
}



// 启动电源
errno_t JakaController::power_on() {
    errno_t ret = robot.power_on(); // 调用底层 SDK 的电源开启函数
    if (ret == ERR_SUCC) {
        std::cout << "Power on successful!" << std::endl;
    }
    else {
        // 打印错误码和对应的错误信息
        std::cerr << "Power on failed with error code: " << ret
            << " (" << get_error_message(ret) << ")" << std::endl;
    }
    return ret;
}

// 使能机器人
errno_t JakaController::enable_robot() {
    errno_t ret = robot.enable_robot(); // 调用底层 SDK 的使能函数
    if (ret == ERR_SUCC) {
        std::cout << "Robot enabled successfully!" << std::endl;
    }
    else {
        std::cerr << "Enable robot failed with error code: " << ret
            << " (" << get_error_message(ret) << ")" << std::endl;
    }
    return ret;
}

// 登出
errno_t JakaController::logout() {
    errno_t ret1 = robot.disable_robot();
    errno_t ret2 = robot.login_out();

    if (ret1 == ERR_SUCC && ret2 == ERR_SUCC) {
        std::cout << "Logout successful!" << std::endl;
    }
    else {
        // 打印错误码和对应的错误信息
        if (ret1 != ERR_SUCC) {
            std::cerr << "Disable robot failed with error code: " << ret1
                << " (" << get_error_message(ret1) << ")" << std::endl;
        }
        if (ret2 != ERR_SUCC) {
            std::cerr << "Login out failed with error code: " << ret2
                << " (" << get_error_message(ret2) << ")" << std::endl;
        }
    }

    return (ret1 == ERR_SUCC && ret2 == ERR_SUCC) ? ERR_SUCC : -1; // 返回综合状态
}



bool JakaController::validate_joint_values(const JointValue* joint_pos) {
    if (!joint_pos) {
        return false; // 指针为空
    }

    // 检查每个关节值是否在范围内
    if (joint_pos->jVal[0] < -360 / 180 * M_PI || joint_pos->jVal[0] > 360 / 180 * M_PI) return false;
    if (joint_pos->jVal[1] < -125 / 180 * M_PI || joint_pos->jVal[1] > 125 / 180 * M_PI) return false;
    if (joint_pos->jVal[2] < -130 / 180 * M_PI || joint_pos->jVal[2] > 130 / 180 * M_PI) return false;
    if (joint_pos->jVal[3] < -360 / 180 * M_PI || joint_pos->jVal[3] > 360 / 180 * M_PI) return false;
    if (joint_pos->jVal[4] < -120 / 180 * M_PI || joint_pos->jVal[4] > 120 / 180 * M_PI) return false;
    if (joint_pos->jVal[5] < -360 / 180 * M_PI || joint_pos->jVal[5] > 360 / 180 * M_PI) return false;

    return true; // 所有关节值均合法
}


errno_t JakaController::joint_move(const JointValue* joint_pos, MoveMode move_mode, BOOL is_block, double speed) {
    // 检查指针是否为空
    if (!joint_pos) {
        std::cerr << "joint_move failed: joint_pos is null!" << std::endl;
        return ERR_INVALID_PARAMETER;
    }

    // 检查速度是否在允许范围内（假设允许速度范围为 0.1 到 max_joint_speed）
    if (speed <= 0 || speed > max_joint_speed) {
        std::cerr << "joint_move failed: speed is out of range! (" << speed << ")" << std::endl;
        return ERR_INVALID_PARAMETER;
    }

    // 如果是绝对模式，直接检查目标关节值是否合法
    if (move_mode == MoveMode::ABS) {
        if (!validate_joint_values(joint_pos)) {
            std::cerr << "joint_move failed: joint values out of range!" << std::endl;
            return ERR_INVALID_PARAMETER;
        }
    }

    // 如果是增量模式，计算增量后的目标值并检查其合法性
    else if (move_mode == MoveMode::INCR) {
        JointValue current_joint_pos;
        errno_t ret = robot.get_joint_position(&current_joint_pos); // 获取当前关节位置
        if (ret != ERR_SUCC) {
            std::cerr << "joint_move failed: unable to get current joint position. Error: "
                << ret << " (" << get_error_message(ret) << ")" << std::endl;
            return ret; // 返回获取关节位置的错误码
        }

        // 计算增量后的目标关节值
        JointValue target_joint_pos;
        target_joint_pos.jVal[0] = current_joint_pos.jVal[0]  + joint_pos->jVal[0];
        target_joint_pos.jVal[1] = current_joint_pos.jVal[1]  + joint_pos->jVal[1];
        target_joint_pos.jVal[2] = current_joint_pos.jVal[2]  + joint_pos->jVal[2];
        target_joint_pos.jVal[3] = current_joint_pos.jVal[3]  + joint_pos->jVal[3];
        target_joint_pos.jVal[4] = current_joint_pos.jVal[4]  + joint_pos->jVal[4];
        target_joint_pos.jVal[5] = current_joint_pos.jVal[5]  + joint_pos->jVal[5];

        // 检查目标值是否在合法范围内
        if (!validate_joint_values(&target_joint_pos)) {
            std::cerr << "joint_move failed: target joint values out of range!" << std::endl;
            return ERR_INVALID_PARAMETER;
        }
    }

    // 调用底层 SDK 的 joint_move 函数
    errno_t ret = robot.joint_move(joint_pos, move_mode, is_block, speed);

    // 检查返回值并处理
    if (ret == ERR_SUCC) {
        std::cout << "joint_move executed successfully." << std::endl;
    }
    else {
        // 打印具体的错误信息
        std::cerr << "joint_move failed with error code: " << ret
            << " (" << get_error_message(ret) << ")" << std::endl;
    }

    return ret; // 返回底层 SDK 的结果
}


errno_t JakaController::get_motion_status(MotionStatus* status) {

    errno_t ret = robot.get_motion_status(status);
    if (ret == ERR_SUCC) {
        std::cout << "get_motion_status executed successfully." << std::endl;
    }
    else {
        // 打印具体的错误信息
        std::cerr << "get_motion_status failed with error code: " << ret
            << " (" << get_error_message(ret) << ")" << std::endl;
    }

    return ret; // 返回底层 SDK 的结果

}


errno_t JakaController::get_robot_status(RobotStatus* status) {

    errno_t ret = robot.get_robot_status(status);

    if (ret == ERR_SUCC) {
        std::cout << "get_robot_status executed successfully." << std::endl;
    }
    else {
        // 打印具体的错误信息
        std::cerr << "get_robot_status failed with error code: " << ret
            << " (" << get_error_message(ret) << ")" << std::endl;
    }

    return ret; // 返回底层 SDK 的结果
}

errno_t JakaController::get_joint_position(JointValue* joint_position) {

    errno_t ret = robot.get_joint_position(joint_position);
    if (ret == ERR_SUCC) {
        std::cout << "get_joint_position executed successfully." << std::endl;
    }
    else {
        // 打印具体的错误信息
        std::cerr << "get_joint_position failed with error code: " << ret
            << " (" << get_error_message(ret) << ")" << std::endl;
    }

    return ret; // 返回底层 SDK 的结果
}