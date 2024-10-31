cmake_minimum_required(VERSION 3.8)
project(basic_control_pkg)

# -------------------------------------
# 设置编译选项
set(CMAKE_CXX_STANDARD 14)

# 包含路径：包含头文件路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/jaka_sdk/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/basic_control_pkg)
# include_directories(
#     include  # 包含自定义头文件
#     include/basic_control_pkg
#     lib/jaka_sdk/include  # 包含 Jaka SDK 头文件
# )

# 链接路径：链接库文件路径
link_directories(
    ${CMAKE_SOURCE_DIR}/lib/jaka_sdk/lib  # 包含 Jaka SDK 库文件
)






# 添加源文件
add_executable(test1 src/test1.cpp )


# 链接 Jaka SDK 库
target_link_libraries(test1 ${CMAKE_SOURCE_DIR}/lib/jaka_sdk/lib/jakaAPI.lib)

# # 假设 jakaAPI.dll 位于 ${CMAKE_SOURCE_DIR}/lib/jaka_sdk/lib/
# add_custom_command(TARGET test1 POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
#         "${CMAKE_SOURCE_DIR}/lib/jaka_sdk/lib/jakaAPI.dll"
#         $<TARGET_FILE_DIR:test1>)  # 复制到可执行文件所在目录



# 安装指令（可选）
install(TARGETS test1 DESTINATION bin)
# -------------------------------------




if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#[[
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
]]