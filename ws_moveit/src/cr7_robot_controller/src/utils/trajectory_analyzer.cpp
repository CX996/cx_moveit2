/**
 * @file trajectory_analyzer.cpp
 * @brief 轨迹分析工具类实现文件
 * 
 * 实现TrajectoryAnalyzer类的所有方法
 */

#include <fstream>
#include <iomanip>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

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

/**
 * @brief 保存详细轨迹分析
 */
void TrajectoryAnalyzer::saveDetailedTrajectoryAnalysis(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& filename_prefix,
    rclcpp::Logger logger) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return;
    }
    
    std::string filename = filename_prefix + "_detailed.txt";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        RCLCPP_WARN(logger, "无法创建详细轨迹分析文件: %s", filename.c_str());
        return;
    }
    
    file << "详细轨迹分析报告" << std::endl;
    file << "====================" << std::endl;
    file << "轨迹点数: " << trajectory.joint_trajectory.points.size() << std::endl;
    
    const auto& points = trajectory.joint_trajectory.points;
    double total_time = points.back().time_from_start.sec + 
                       points.back().time_from_start.nanosec * 1e-9;
    file << "轨迹总时间: " << std::fixed << std::setprecision(3) << total_time << " 秒" << std::endl;
    
    if (!trajectory.joint_trajectory.joint_names.empty()) {
        file << "关节名称: " << std::endl;
        for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
            file << "  " << i << ": " << trajectory.joint_trajectory.joint_names[i] << std::endl;
        }
    }
    
    file << "\n详细轨迹点信息:" << std::endl;
    file << "================" << std::endl;
    
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        double time_from_start = point.time_from_start.sec + 
                                point.time_from_start.nanosec * 1e-9;
        
        file << "轨迹点 " << i << ":" << std::endl;
        file << "  时间: " << std::fixed << std::setprecision(6) << time_from_start << " 秒" << std::endl;
        
        if (!point.positions.empty()) {
            file << "  关节位置: [";
            for (size_t j = 0; j < point.positions.size(); ++j) {
                file << std::fixed << std::setprecision(6) << point.positions[j];
                if (j < point.positions.size() - 1) {
                    file << ", ";
                }
            }
            file << "]" << std::endl;
        }
        
        if (!point.velocities.empty()) {
            file << "  关节速度: [";
            for (size_t j = 0; j < point.velocities.size(); ++j) {
                file << std::fixed << std::setprecision(6) << point.velocities[j];
                if (j < point.velocities.size() - 1) {
                    file << ", ";
                }
            }
            file << "]" << std::endl;
        }
        
        if (!point.accelerations.empty()) {
            file << "  关节加速度: [";
            for (size_t j = 0; j < point.accelerations.size(); ++j) {
                file << std::fixed << std::setprecision(6) << point.accelerations[j];
                if (j < point.accelerations.size() - 1) {
                    file << ", ";
                }
            }
            file << "]" << std::endl;
        }
        
        file << std::endl;
    }
    
    file.close();
    RCLCPP_INFO(logger, "详细轨迹分析已保存到: %s", filename.c_str());
}

/**
 * @brief 检查轨迹是否是直线
 */
bool TrajectoryAnalyzer::isTrajectoryLinear(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& end_pose,
    double max_deviation,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    rclcpp::Logger logger)
{
    if (trajectory.joint_trajectory.points.empty())
    {
        return false;
    }
    
    // 获取轨迹中的位姿点
    std::vector<geometry_msgs::msg::Pose> trajectory_poses;
    
    try
    {
        // 对于每个关节轨迹点，计算末端执行器的位姿
        for (const auto& point : trajectory.joint_trajectory.points)
        {
            // 设置关节位置
            std::vector<double> joint_values = point.positions;
            move_group->setJointValueTarget(joint_values);
            
            // 计算正向运动学
            moveit::core::RobotStatePtr kinematic_state = move_group->getCurrentState();
            kinematic_state->setJointGroupPositions(move_group->getName(), joint_values);
            
            // 获取末端执行器位姿
            geometry_msgs::msg::PoseStamped end_effector_pose;
            try
            {
                const std::string& end_effector_link = move_group->getEndEffectorLink();
                kinematic_state->getGlobalLinkTransform(end_effector_link).copyToPose(end_effector_pose.pose);
                end_effector_pose.header.frame_id = move_group->getPlanningFrame();
                trajectory_poses.push_back(end_effector_pose.pose);
            }
            catch (...)
            {
                // 跳过无法计算的点
                continue;
            }
        }
    }
    catch (...)
    {
        RCLCPP_WARN(logger, "计算轨迹位姿失败");
        return false;
    }
    
    if (trajectory_poses.size() < 2)
    {
        return false;
    }
    
    // 计算直线向量
    Eigen::Vector3d start_point(
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z
    );
    
    Eigen::Vector3d end_point(
        end_pose.position.x,
        end_pose.position.y,
        end_pose.position.z
    );
    
    Eigen::Vector3d line_vector = end_point - start_point;
    double line_length = line_vector.norm();
    
    if (line_length < 1e-6)
    {
        // 起点和终点重合，视为直线
        return true;
    }
    
    line_vector.normalize();
    
    // 检查每个点到直线的距离
    for (const auto& pose : trajectory_poses)
    {
        Eigen::Vector3d point(
            pose.position.x,
            pose.position.y,
            pose.position.z
        );
        
        // 计算点到直线的向量
        Eigen::Vector3d point_vector = point - start_point;
        
        // 计算点到直线的距离
        double distance = (point_vector - point_vector.dot(line_vector) * line_vector).norm();
        
        if (distance > max_deviation)
        {
            RCLCPP_WARN(logger, "轨迹偏离直线: 距离 = %.6f m, 最大允许 = %.6f m", distance, max_deviation);
            return false;
        }
    }
    
    RCLCPP_INFO(logger, "轨迹是直线，最大偏差小于 %.6f m", max_deviation);
    return true;
}

} // namespace utils
} // namespace cr7_controller