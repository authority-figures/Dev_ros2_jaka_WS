
#include "JAKAZuRobot.h"
#include "JakaController.h"
#include "ServoController.h"

int main() {
    // 初始化机械臂接口
    //JAKAZuRobot robot;
    //robot.login_in("10.5.5.100");
    JakaController controller("10.5.5.100");
    controller.login();
    controller.power_on();
    controller.enable_robot();
    // 定义关节目标值
    JointValue joint_pos = {
        0 , // joint1
        0 , // joint2
        0 , // joint3
        0 , // joint4
        0 , // joint5
        20 // joint6
    };

    // 调用 joint_move
    int i = 1;
    while (true)
    {   
        i = -i;
        joint_pos.jVal[5] = -joint_pos.jVal[5];
        errno_t ret = controller.joint_move(&joint_pos, MoveMode::INCR, true, 5);
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    return 0;
}
