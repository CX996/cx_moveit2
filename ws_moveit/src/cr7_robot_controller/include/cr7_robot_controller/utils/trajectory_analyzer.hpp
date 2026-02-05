/**
 * @file trajectory_analyzer.hpp
 * @brief 轨迹分析工具类头文件
 * 
 * 这个文件定义了轨迹分析相关的工具函数，包括：
 * 1. 轨迹分析和保存
 * 2. 路径点分析和保存
 * 3. 轨迹信息打印
 */

#ifndef TRAJECTORY_ANALYZER_HPP_
#define TRAJECTORY_ANALYZER_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace cr7_controller {
namespace utils {

/**
 * @class TrajectoryAnalyzer
 * @brief 轨迹分析工具类
 * 
 * 这个类提供轨迹分析相关的静态工具函数
 */
class TrajectoryAnalyzer {
public:
    /**
     * @brief 保存轨迹分析
     * @param trajectory 机器人轨迹
     * @param filename_prefix 文件名前缀
     * @param logger ROS logger
     */
    static void saveTrajectoryAnalysis(
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        const std::string& filename_prefix,
        rclcpp::Logger logger);
    
    /**
     * @brief 保存路径点分析
     * @param waypoints 路径点序列
     * @param filename_prefix 文件名前缀
     * @param logger ROS logger
     */
    static void saveWaypointAnalysis(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        const std::string& filename_prefix,
        rclcpp::Logger logger);
    
    /**
     * @brief 打印轨迹信息
     * @param trajectory 机器人轨迹
     * @param logger ROS logger
     */
    static void printTrajectoryInfo(
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        rclcpp::Logger logger);
};

} // namespace utils
} // namespace cr7_controller

#endif // TRAJECTORY_ANALYZER_HPP_
