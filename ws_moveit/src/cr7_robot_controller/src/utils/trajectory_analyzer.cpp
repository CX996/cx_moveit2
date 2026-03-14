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

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace cr7_controller {
namespace utils {

namespace {
static double toSec(const builtin_interfaces::msg::Duration& t)
{
    return static_cast<double>(t.sec) +
           static_cast<double>(t.nanosec) * 1e-9;
}
}

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


 /**
 * @brief 使用五次多项式对关节轨迹进行重采样
 * 
 * 该函数使用五次多项式插值方法对输入轨迹进行重采样，生成指定时间步长的新轨迹。
 * 五次多项式能够保证位置、速度和加速度的连续性，提供平滑的轨迹过渡。
 * 
 * 特性：
 *  - 使用 Quintic Polynomial (五次多项式)
 *  - 保证 C2 连续 (位置、速度、加速度连续)
 *  - 保留原始轨迹的边界条件
 *  - 生成固定时间步长轨迹
 *  - 确保终点速度和加速度为0（机器人静止）
 * 
 * 适用于：
 *  - 工业机器人控制
 *  - ServoJ streaming
 *  - 轨迹控制周期匹配
 *  - 轨迹平滑处理
 * 
 * 算法原理：
 * 1. 对轨迹按时间步长 dt 进行均匀采样
 * 2. 对每个采样点，确定其所在的轨迹段
 * 3. 使用五次多项式插值计算该点的位置、速度和加速度
 * 4. 确保轨迹终点的速度和加速度为0
 * 
 * 五次多项式插值公式：
 * q(t) = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
 * v(t) = c1 + 2*c2*t + 3*c3*t² + 4*c4*t³ + 5*c5*t⁴
 * a(t) = 2*c2 + 6*c3*t + 12*c4*t² + 20*c5*t³
 * 
 * 其中系数 c0-c5 通过边界条件（起始和结束的位置、速度、加速度）计算得出
 * 
 * @param input_traj 输入的关节轨迹
 * @param dt 重采样的时间步长（秒），建议值：0.01-0.05秒
 * @return 重采样后的关节轨迹
 */
trajectory_msgs::msg::JointTrajectory TrajectoryAnalyzer::resampleTrajectory(
    const trajectory_msgs::msg::JointTrajectory& input_traj,
    double dt)
{
    // 创建输出轨迹
    trajectory_msgs::msg::JointTrajectory output;

    // 处理边界情况：轨迹点少于2个时直接返回原轨迹
    if (input_traj.points.size() < 2)
        return input_traj;

    // 复制关节名称信息
    output.joint_names = input_traj.joint_names;

    // 计算轨迹总时间
    double total_time = toSec(input_traj.points.back().time_from_start);

    // 当前轨迹段索引
    size_t segment = 0;

    // 按指定时间步长逐点采样
    for (double t = 0.0; t < total_time; t += dt)
    {
        // 找到当前时间所在的轨迹段
        // 注意：使用 input_traj.points.size() - 2 作为上界，确保 segment + 1 不会越界
        while (segment < input_traj.points.size() - 2 &&
               t > toSec(input_traj.points[segment + 1].time_from_start))
        {
            segment++;
        }

        // 获取当前轨迹段的起点和终点
        const auto& p0 = input_traj.points[segment];
        const auto& p1 = input_traj.points[segment + 1];

        // 计算当前轨迹段的时间信息
        double t0 = toSec(p0.time_from_start);  // 轨迹段起始时间
        double t1 = toSec(p1.time_from_start);  // 轨迹段结束时间
        double T = t1 - t0;                     // 轨迹段持续时间
        double tau = t - t0;                    // 当前时间在轨迹段内的相对时间

        // 创建新的轨迹点
        trajectory_msgs::msg::JointTrajectoryPoint new_point;

        // 获取自由度数量
        size_t dof = p0.positions.size();

        // 为新轨迹点分配空间
        new_point.positions.resize(dof);
        new_point.velocities.resize(dof);
        new_point.accelerations.resize(dof);

        // 对每个关节进行五次多项式插值
        for (size_t j = 0; j < dof; j++)
        {
            // 提取边界条件
            double q0 = p0.positions[j];          // 起始位置
            double q1 = p1.positions[j];          // 结束位置
            double v0 = p0.velocities.empty() ? 0.0 : p0.velocities[j];  // 起始速度
            double v1 = p1.velocities.empty() ? 0.0 : p1.velocities[j];  // 结束速度
            double a0 = p0.accelerations.empty() ? 0.0 : p0.accelerations[j];  // 起始加速度
            double a1 = p1.accelerations.empty() ? 0.0 : p1.accelerations[j];  // 结束加速度

            // 计算五次多项式系数
            double c0 = q0;                          // 常数项（起始位置）
            double c1 = v0;                          // 一次项系数（起始速度）
            double c2 = a0 / 2.0;                    // 二次项系数（起始加速度）

            // 预计算时间相关的幂次，提高计算效率
            double T2 = T*T;
            double T3 = T2*T;
            double T4 = T3*T;
            double T5 = T4*T;

            // 计算三次项系数
            double c3 =
                (20*(q1-q0) - (8*v1+12*v0)*T - (3*a0-a1)*T2) / (2*T3);

            // 计算四次项系数
            double c4 =
                (30*(q0-q1) + (14*v1+16*v0)*T + (3*a0-2*a1)*T2) / (2*T4);

            // 计算五次项系数
            double c5 =
                (12*(q1-q0) - (6*v1+6*v0)*T - (a0-a1)*T2) / (2*T5);

            // 预计算相对时间的幂次，提高计算效率
            double tau2 = tau*tau;
            double tau3 = tau2*tau;
            double tau4 = tau3*tau;
            double tau5 = tau4*tau;

            // 计算当前时间点的位置
            new_point.positions[j] =
                c0 +
                c1*tau +
                c2*tau2 +
                c3*tau3 +
                c4*tau4 +
                c5*tau5;

            // 计算当前时间点的速度（位置对时间的一阶导数）
            new_point.velocities[j] =
                c1 +
                2*c2*tau +
                3*c3*tau2 +
                4*c4*tau3 +
                5*c5*tau4;

            // 计算当前时间点的加速度（位置对时间的二阶导数）
            new_point.accelerations[j] =
                2*c2 +
                6*c3*tau +
                12*c4*tau2 +
                20*c5*tau3;
        }

        // 设置新轨迹点的时间戳
        new_point.time_from_start =
            rclcpp::Duration::from_seconds(t);

        // 将新轨迹点添加到输出轨迹中
        output.points.push_back(new_point);
    }

    // 确保输出轨迹的终点与输入轨迹完全一致
    // 这是为了避免由于浮点数计算误差导致的终点偏差
    // 同时确保终点的速度和加速度都为0（机器人应该在终点静止）
    trajectory_msgs::msg::JointTrajectoryPoint end_point = input_traj.points.back();
    for (size_t j = 0; j < end_point.positions.size(); j++)
    {
        if (!end_point.velocities.empty())
            end_point.velocities[j] = 0.0;
        if (!end_point.accelerations.empty())
            end_point.accelerations[j] = 0.0;
    }
    output.points.push_back(end_point);

    return output;
}

} // namespace utils
} // namespace cr7_controller