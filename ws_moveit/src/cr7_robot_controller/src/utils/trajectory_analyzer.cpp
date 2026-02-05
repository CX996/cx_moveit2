/**
 * @file trajectory_analyzer.cpp
 * @brief 轨迹分析工具类实现文件
 * 
 * 实现TrajectoryAnalyzer类的所有方法
 */

#include <fstream>
#include <iomanip>

#include "cr7_robot_controller/utils/trajectory_analyzer.hpp"

namespace cr7_controller {
namespace utils {

/**
 * @brief 保存轨迹分析
 */
void TrajectoryAnalyzer::saveTrajectoryAnalysis(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& filename_prefix,
    rclcpp::Logger logger) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return;
    }
    
    std::string filename = filename_prefix + ".txt";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        RCLCPP_WARN(logger, "无法创建轨迹分析文件: %s", filename.c_str());
        return;
    }
    
    file << "轨迹分析报告" << std::endl;
    file << "================" << std::endl;
    file << "轨迹点数: " << trajectory.joint_trajectory.points.size() << std::endl;
    
    const auto& points = trajectory.joint_trajectory.points;
    double total_time = points.back().time_from_start.sec + 
                       points.back().time_from_start.nanosec * 1e-9;
    file << "轨迹总时间: " << std::fixed << std::setprecision(3) << total_time << " 秒" << std::endl;
    
    file.close();
    RCLCPP_INFO(logger, "轨迹分析已保存到: %s", filename.c_str());
}

/**
 * @brief 保存路径点分析
 */
void TrajectoryAnalyzer::saveWaypointAnalysis(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const std::string& filename_prefix,
    rclcpp::Logger logger) 
{
    
    if (waypoints.empty()) 
    {
        return;
    }
    
    std::string filename = filename_prefix + ".txt";
    std::ofstream file(filename);
    
    if (!file.is_open()) 
    {
        RCLCPP_WARN(logger, "无法创建路径点分析文件: %s", filename.c_str());
        return;
    }
    
    file << "路径点分析报告" << std::endl;
    file << "================" << std::endl;
    file << "路径点数量: " << waypoints.size() << std::endl;
    
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        const auto& pose = waypoints[i];
        file << "路点 " << i << ":" << std::endl;
        file << "  位置: [" << std::fixed << std::setprecision(6) 
             << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "]" << std::endl;
        file << "  姿态: [" << std::fixed << std::setprecision(6) 
             << pose.orientation.x << ", " << pose.orientation.y << ", " 
             << pose.orientation.z << ", " << pose.orientation.w << "]" << std::endl;
    }
    
    file.close();
    RCLCPP_INFO(logger, "路径点分析已保存到: %s", filename.c_str());
}

/**
 * @brief 打印轨迹信息
 */
void TrajectoryAnalyzer::printTrajectoryInfo(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    rclcpp::Logger logger) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return;
    }
    
    const auto& points = trajectory.joint_trajectory.points;
    
    RCLCPP_INFO(logger, "轨迹详细信息:");
    RCLCPP_INFO(logger, "- 轨迹点数: %zu", points.size());
    
    // 打印轨迹的起始和结束时间
    double start_time = points[0].time_from_start.sec + 
                       points[0].time_from_start.nanosec * 1e-9;
    double end_time = points.back().time_from_start.sec + 
                     points.back().time_from_start.nanosec * 1e-9;
    
    RCLCPP_INFO(logger, "- 起始时间: %.3f 秒", start_time);
    RCLCPP_INFO(logger, "- 结束时间: %.3f 秒", end_time);
    RCLCPP_INFO(logger, "- 总时间: %.3f 秒", end_time - start_time);
}

} // namespace utils
} // namespace cr7_controller
