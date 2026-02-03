# CMake最小版本要求
cmake_minimum_required(VERSION 3.8)
project(cr7_robot_controller)

# 如果没有设置C++标准，设置为C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 设置C++标准为必需
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 包含目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(cr7_controller
  src/cr7_robot_controller.cpp
  src/main.cpp
)

# 链接依赖
ament_target_dependencies(cr7_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# 在target_link_libraries中添加
target_link_libraries(cr7_controller
  ${MOVEIT_LIBRARIES}
  tf2::tf2
  tf2_ros::tf2_ros
)

# 安装目标
install(TARGETS cr7_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# 生成ament包
ament_package()