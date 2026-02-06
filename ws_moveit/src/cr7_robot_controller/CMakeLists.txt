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

# 共享源文件（所有可执行文件都需要）
set(SHARED_SOURCE_FILES
  src/cr7_robot_controller.cpp
  src/base/cr7_base_controller.cpp
  src/planners/ompl/cr7_ompl_planner.cpp
  src/planners/cartesian/cr7_cartesian_planner.cpp
  src/planners/pilz/cr7_pilz_planner.cpp
  src/path/cr7_path_executor.cpp
  # 确保包含 utils 目录下的文件
  src/utils/pose_utils.cpp
  src/utils/trajectory_analyzer.cpp
)

# 创建主可执行文件
add_executable(cr7_controller
  ${SHARED_SOURCE_FILES}
  src/main.cpp
)

# 链接依赖
ament_target_dependencies(cr7_controller
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2_geometry_msgs
)

# 在target_link_libraries中添加
target_link_libraries(cr7_controller
  ${MOVEIT_LIBRARIES}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
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
