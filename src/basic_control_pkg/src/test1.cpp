#include <stdio.h>
#include <iostream>

#include "JAKAZuRobot.h"

int main() {
    // 初始化机械臂接口
    
    JAKAZuRobot robot;
    robot.login_in("10.5.5.100");
    std::cout << "1" << std::endl;
    // 暂停系统
    system("pause");

    return 0;
}
