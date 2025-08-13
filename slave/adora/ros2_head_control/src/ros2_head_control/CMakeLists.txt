cmake_minimum_required(VERSION 3.8)
project(ros2_head_control)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_head_msgs REQUIRED)  # 新消息包依赖

# 安装Python包
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  src/${PROJECT_NAME}/head_control_node.py
  src/${PROJECT_NAME}/motor_status_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclpy
  ros2_head_msgs  # 导出消息包依赖
)

ament_package()