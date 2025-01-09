

# 指定 C++ 标准
set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.10)



project(basic_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/Project/ROS2_project/Jaka_workspace/Dev_ros2_jaka_WS/install

# set(CMAKE_PREFIX_PATH "~/Project/ROS2_project/Jaka_workspace/Dev_ros2_jaka_WS/install" ${CMAKE_PREFIX_PATH})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(rosidl_default_generators REQUIRED )
find_package(my_custom_msgs REQUIRED)


# # 生成消息
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/RobotStatus.msg"
#   DEPENDENCIES sensor_msgs
# )


# find_package(<dependency> REQUIRED)
# 系统路径中的头文件
# 声明库文件路径
# 添加库文件路径
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)


# 包含头文件路径
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/basic_control_pkg
  ${catkin_INCLUDE_DIRS}
)

# 可执行文件目标
add_executable(
  joint_move_node
  src/JakaController.cpp
  src/ServoController.cpp
  src/joint_move_node.cpp
  # src/源.cpp
)






find_library(JAKA_API_LIBRARY NAMES jakaAPI PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 链接库
target_link_libraries(
  joint_move_node

  ${JAKA_API_LIBRARY}
)

ament_target_dependencies(joint_move_node rclcpp std_msgs sensor_msgs 
  my_custom_msgs
)
install(TARGETS
  joint_move_node
  # my_custom_msgs
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  FILES ${JAKA_API_LIBRARY} DESTINATION lib
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
