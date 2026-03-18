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
#include <builtin_interfaces/msg/duration.hpp>

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
 * @brief 计算四元数连续性（检查相邻点间的四元数突变）
 * @param prev_q 上一个四元数
 * @param curr_q 当前四元数
 * @return 如果四元数符号有跳变，返回调整后的当前四元数
 */
Eigen::Quaterniond ensureQuaternionContinuity(
    const Eigen::Quaterniond& prev_q, 
    const Eigen::Quaterniond& curr_q)
{
    // 计算点积
    double dot = prev_q.w() * curr_q.w() + 
                prev_q.x() * curr_q.x() + 
                prev_q.y() * curr_q.y() + 
                prev_q.z() * curr_q.z();
    
    // 如果点积为负，说明两个四元数表示相反的旋转方向
    // 将它们取为同一半球以保证连续性
    if (dot < 0) {
        // 返回取反的四元数
        return Eigen::Quaterniond(-curr_q.w(), -curr_q.x(), -curr_q.y(), -curr_q.z());
    }
    
    return curr_q;
}

/**
 * @brief 保存详细轨迹分析
 */
void TrajectoryAnalyzer::saveDetailedTrajectoryAnalysis(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& filename_prefix,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    rclcpp::Logger logger) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return;
    }
    
    // 定义日志文件保存路径
    std::string log_path = "src/cr7_robot_controller/data/log/";  // 设置您的日志路径

    std::string filename = log_path + filename_prefix + "_detailed.txt";
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
    
    // 添加诊断信息说明
    file << "\n注意：四元数(qx, qy, qz, qw)表示姿态。" << std::endl;
    file << "四元数q和-q表示相同的旋转，但会导致数值突变。" << std::endl;
    file << "在轨迹插值中可能出现符号跳变，这属于正常现象。" << std::endl;
    
    file << "\n详细轨迹点信息:" << std::endl;
    file << "================" << std::endl;
    
    // 用于跟踪前一个四元数
    Eigen::Quaterniond prev_quat;
    bool has_prev_quat = false;
    int discontinuity_count = 0;
    
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
        
        // 计算并记录笛卡尔坐标位置
        if (move_group) {
            try {
                // 设置关节位置
                std::vector<double> joint_values = point.positions;
                move_group->setJointValueTarget(joint_values);
                
                // 计算正向运动学
                moveit::core::RobotStatePtr kinematic_state = move_group->getCurrentState();
                kinematic_state->setJointGroupPositions(move_group->getName(), joint_values);
                
                // 获取末端执行器位姿
                const std::string& end_effector_link = move_group->getEndEffectorLink();
                Eigen::Isometry3d transform = kinematic_state->getGlobalLinkTransform(end_effector_link);
                
                // 从 Eigen::Isometry3d 转换到 geometry_msgs::msg::Pose
                double x = transform.translation().x();
                double y = transform.translation().y();
                double z = transform.translation().z();
                
                Eigen::Quaterniond q(transform.rotation());
                q.normalize();  // 确保是单位四元数
                
                // 检查连续性
                bool discontinuity_detected = false;
                if (has_prev_quat) {
                    double dot = prev_quat.w() * q.w() + 
                                prev_quat.x() * q.x() + 
                                prev_quat.y() * q.y() + 
                                prev_quat.z() * q.z();
                    
                    if (dot < 0) {
                        discontinuity_detected = true;
                        discontinuity_count++;
                        file << "  [注意] 检测到四元数符号跳变！" << std::endl;
                        file << "         与前一帧点积: " << std::setprecision(6) << dot << std::endl;
                        
                        // 自动调整到同一半球
                        q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
                        file << "         已自动调整符号以保证连续性" << std::endl;
                    }
                }
                
                // // 确保四元数在半球表示（w为正）
                // if (q.w() < 0) {
                //     q = Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
                // }
                
                // 保存为当前的四元数用于下一帧比较
                prev_quat = q;
                has_prev_quat = true;
                
                double qx = q.x();
                double qy = q.y();
                double qz = q.z();
                double qw = q.w();
                
                file << "  笛卡尔位置: [" << std::fixed << std::setprecision(6)
                     << x << ", " << y << ", " << z << "]" << std::endl;
                file << "  笛卡尔姿态: [" << std::fixed << std::setprecision(6)
                     << qx << ", " << qy << ", " << qz << ", " << qw << "]" << std::endl;
                
                // 可选的：添加欧拉角表示（更直观）
                Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX顺序
                file << "  欧拉角(ZYX): [" << std::fixed << std::setprecision(6)
                     << euler[0] << ", " << euler[1] << ", " << euler[2] << "] 弧度" << std::endl;
                
            } catch (...) {
                // 跳过无法计算的点
                file << "  笛卡尔位置: 无法计算" << std::endl;
            }
        }
        
        file << std::endl;
    }
    
    // 添加诊断总结
    if (discontinuity_count > 0) {
        file << "\n诊断信息:" << std::endl;
        file << "================" << std::endl;
        file << "检测到 " << discontinuity_count << " 次四元数符号跳变。" << std::endl;
        file << "这是正常现象，因为四元数 q 和 -q 表示相同的旋转。" << std::endl;
        file << "在插值算法中可能会产生这种跳变。" << std::endl;
    }
    
    file.close();
    RCLCPP_INFO(logger, "详细轨迹分析已保存到: %s", filename.c_str());
    
    if (discontinuity_count > 0) {
        RCLCPP_WARN(logger, "检测到 %d 次四元数符号跳变，已自动调整", discontinuity_count);
    }
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
                Eigen::Isometry3d transform = kinematic_state->getGlobalLinkTransform(end_effector_link);
                
                // 从 Eigen::Isometry3d 转换到 geometry_msgs::msg::Pose
                end_effector_pose.pose.position.x = transform.translation().x();
                end_effector_pose.pose.position.y = transform.translation().y();
                end_effector_pose.pose.position.z = transform.translation().z();
                
                Eigen::Quaterniond q(transform.rotation());
                end_effector_pose.pose.orientation.x = q.x();
                end_effector_pose.pose.orientation.y = q.y();
                end_effector_pose.pose.orientation.z = q.z();
                end_effector_pose.pose.orientation.w = q.w();
                
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
    
    double distance = 0.0;
    int iCount = 0;
    int iErrorCount = 0;
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
        iCount++;
        
        // 计算点到直线的距离
        distance = (point_vector - point_vector.dot(line_vector) * line_vector).norm();
        
        if (distance > max_deviation)
        {
            RCLCPP_WARN(logger, "轨迹偏离直线: 距离 = %.6f m, 最大允许 = %.6f m, 错误点位 = %d", distance, max_deviation, iCount);
            iErrorCount++;
        }
        else
        {
            // RCLCPP_INFO(logger, "轨迹点距离直线: %.6f m (在允许范围内)", distance);
        }

        if (iErrorCount > 10) // 如果错误点超过10个，认为轨迹不是直线
        {
            RCLCPP_WARN(logger, "轨迹偏离直线过多，认为不是直线轨迹");
            return false;
        }
    }
    
    RCLCPP_INFO(logger, "轨迹是直线，最大偏差小于 %.6f m", max_deviation);
    return true;
}

} // namespace utils
} // namespace cr7_controller