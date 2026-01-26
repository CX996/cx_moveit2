#include <memory>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class MultiWaypointController {
public:
  MultiWaypointController() 
    : node_(std::make_shared<rclcpp::Node>("multi_waypoint_moveit")),
      move_group_interface_(node_, "panda_arm") {
  }

  // 通过位置和四元数移动到单个点位
  bool moveToWaypoint(double x, double y, double z, 
                      double qx, double qy, double qz, double qw,
                      const std::string& name = "") {
    
    auto logger = node_->get_logger();
    
    RCLCPP_INFO(logger, "Moving to: %s (pos: %.3f, %.3f, %.3f, quat: %.3f, %.3f, %.3f, %.3f)", 
                name.c_str(), x, y, z, qx, qy, qz, qw);
    
    // 创建位姿
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    
    target_pose.orientation.x = qx;
    target_pose.orientation.y = qy;
    target_pose.orientation.z = qz;
    target_pose.orientation.w = qw;
    
    // 调试输出
    RCLCPP_INFO(logger, "Target Pose: x=%.3f, y=%.3f, z=%.3f, qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                target_pose.position.x, target_pose.position.y, target_pose.position.z,
                target_pose.orientation.x, target_pose.orientation.y, 
                target_pose.orientation.z, target_pose.orientation.w);
    
    move_group_interface_.setPoseTarget(target_pose);
    
    // 规划并执行
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface_.plan(plan));
    
    if (success) {
      RCLCPP_INFO(logger, "Planning succeeded, executing...");
      move_group_interface_.execute(plan);
      RCLCPP_INFO(logger, "Reached: %s", name.c_str());
      return true;
    } else {
      RCLCPP_ERROR(logger, "Failed to move to: %s", name.c_str());
      return false;
    }
  }

  // 通过Pose消息移动到单个点位
  bool moveToPose(const geometry_msgs::msg::Pose& pose, const std::string& name = "") {
    return moveToWaypoint(pose.position.x, pose.position.y, pose.position.z,
                          pose.orientation.x, pose.orientation.y, 
                          pose.orientation.z, pose.orientation.w, name);
  }

  // 定义Pose结构体方便使用
  struct WaypointPose {
    double x, y, z;
    double qx, qy, qz, qw;
    std::string name;
    
    // 转换为geometry_msgs::msg::Pose
    geometry_msgs::msg::Pose toPoseMsg() const {
      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation.x = qx;
      pose.orientation.y = qy;
      pose.orientation.z = qz;
      pose.orientation.w = qw;
      return pose;
    }
  };

  // 通过WaypointPose移动到单个点位
  bool moveToWaypointPose(const WaypointPose& wp) {
    return moveToWaypoint(wp.x, wp.y, wp.z, wp.qx, wp.qy, wp.qz, wp.qw, wp.name);
  }

  // 移动到多个点位
  void moveThroughWaypoints(const std::vector<WaypointPose>& waypoints, 
                           double delay_seconds = 1.0) {
    
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "Starting to move through %zu waypoints", waypoints.size());
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto& wp = waypoints[i];
      
      RCLCPP_INFO(logger, "Moving to waypoint %zu/%zu: %s", 
                  i+1, waypoints.size(), wp.name.c_str());
      
      if (!moveToWaypointPose(wp)) {
        RCLCPP_ERROR(logger, "Stopping due to failure at waypoint %zu: %s", 
                    i + 1, wp.name.c_str());
        break;
      }
      
      // 点位间延迟
      if (i < waypoints.size() - 1) {
        RCLCPP_INFO(logger, "Waiting %.1f seconds before next waypoint...", delay_seconds);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(delay_seconds * 1000)));
      }
    }
    
    RCLCPP_INFO(logger, "Waypoint sequence completed");
  }

  // 执行预设路径
  void executeDemoPath() {
    // 定义多个焊点位姿
    // 注意：四元数应该是一个单位四元数（qx^2 + qy^2 + qz^2 + qw^2 = 1）
    std::vector<WaypointPose> waypoints = {
      // 第一个焊点
      {0.76908, -0.38109, 0.036355, 
       0.72419, 0.62801, 0.10427, 0.26512,  // 单位四元数，无旋转
       "Weld_Point_1"},
      
      // 第二个焊点
      {0.77031, 0.37564, 0.039695, 
       -0.61661, 0.72446, -0.13272, 0.2781,  // 绕Z轴旋转180度
       "Weld_Point_2"},
    };
    
    // 验证四元数是否为单位四元数
    auto logger = node_->get_logger();
    for (const auto& wp : waypoints) {
      double norm = sqrt(wp.qx*wp.qx + wp.qy*wp.qy + wp.qz*wp.qz + wp.qw*wp.qw);
      RCLCPP_INFO(logger, "Waypoint %s quaternion norm: %.6f", 
                  wp.name.c_str(), norm);
      if (abs(norm - 1.0) > 0.001) {
        RCLCPP_WARN(logger, "Warning: Quaternion for %s is not normalized! norm=%.6f", 
                    wp.name.c_str(), norm);
      }
    }
    
    moveThroughWaypoints(waypoints, 2.0);  // 2秒延迟
  }

  // 获取当前末端位姿
  geometry_msgs::msg::Pose getCurrentPose() {
    return move_group_interface_.getCurrentPose().pose;
  }
  
  // 显示当前位姿
  void printCurrentPose() {
    auto current_pose = getCurrentPose();
    auto logger = node_->get_logger();
    RCLCPP_INFO(logger, "Current Pose:");
    RCLCPP_INFO(logger, "  Position:  x=%.3f, y=%.3f, z=%.3f",
                current_pose.position.x, current_pose.position.y, current_pose.position.z);
    RCLCPP_INFO(logger, "  Orientation: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                current_pose.orientation.x, current_pose.orientation.y,
                current_pose.orientation.z, current_pose.orientation.w);
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  MultiWaypointController controller;
  
  // 显示初始位姿
  controller.printCurrentPose();
  
  // 方法1: 执行预设焊接路径
  controller.executeDemoPath();
  
  // 方法2: 使用简单的四元数
  // 单位四元数示例：
  // 1. 无旋转: (0, 0, 0, 1)
  // 2. 绕Z轴旋转180度: (0, 0, 1, 0) 或 (0, 0, 0.707, 0.707) 对应90度
  // 3. 绕Y轴旋转180度: (0, 1, 0, 0)
  // 4. 绕X轴旋转180度: (1, 0, 0, 0)
  
  /*
  std::vector<MultiWaypointController::WaypointPose> simpleWaypoints = {
    {0.3, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0, "Home"},  // 无旋转
    {0.3, -0.2, 0.5, 0.0, 0.707, 0.0, 0.707, "Point1"},  // 绕Y轴旋转90度
    {0.1, -0.2, 0.4, 0.0, 0.0, 0.707, 0.707, "Point2"},  // 绕Z轴旋转90度
  };
  controller.moveThroughWaypoints(simpleWaypoints, 1.0);
  */
  
  // 显示最终位姿
  controller.printCurrentPose();
  
  rclcpp::shutdown();
  return 0;
}