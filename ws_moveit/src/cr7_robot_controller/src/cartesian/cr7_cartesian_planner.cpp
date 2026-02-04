/**
 * @file cr7_cartesian_planner.cpp
 * @brief 笛卡尔路径规划模块实现文件
 * 
 * 实现CR7CartesianPlanner类的所有方法
 */


#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <fstream>

#include "cr7_robot_controller/cartesian/cr7_cartesian_planner.hpp"

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7CartesianPlanner 构造函数
// ============================================================================
/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param planning_group MoveIt规划组名称
 */
CR7CartesianPlanner::CR7CartesianPlanner(
    rclcpp::Node::SharedPtr node,
    const std::string& planning_group
) : CR7BaseController(node, planning_group), config_()
{
    RCLCPP_INFO(logger_, "创建笛卡尔路径规划器");
}

// ============================================================================
// 笛卡尔路径规划核心实现
// ============================================================================

/**
 * @brief 笛卡尔路径规划
 * @param waypoints 路径点序列
 * @param config 笛卡尔路径配置
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7CartesianPlanner::executeCartesianPath(
    const std::vector<Waypoint>& waypoints,
    const CartesianPathConfig& config) 
{
    
    if (!move_group_)
    {
        RCLCPP_ERROR(logger_, "MoveGroup接口未初始化");
        return CR7BaseController::Result::ROBOT_NOT_READY;
    }
    
    if (waypoints.size() < 2)
    {
        RCLCPP_ERROR(logger_, "笛卡尔路径需要至少2个路点");
        return CR7BaseController::Result::INVALID_INPUT;
    }
    
    // 获取规划坐标系
    std::string planning_frame = move_group_->getPlanningFrame();
    std::vector<geometry_msgs::msg::Pose> waypoint_poses;

    // 变换所有路点到规划坐标系
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        const auto& wp = waypoints[i];
        
        // 创建PoseStamped
        geometry_msgs::msg::PoseStamped input_pose;
        input_pose.header.frame_id = wp.frame_id;
        input_pose.header.stamp = node_->now();
        input_pose.pose = wp.toPose();
        
        // 转换到规划坐标系
        geometry_msgs::msg::PoseStamped output_pose;
        if (transformPose(input_pose, output_pose, planning_frame)) 
        {
            waypoint_poses.push_back(output_pose.pose);
            RCLCPP_INFO(logger_, "路点 %zu 坐标变换成功: %s -> %s", 
                       i, wp.frame_id.c_str(), planning_frame.c_str());
        } 
        else
        {
            RCLCPP_ERROR(logger_, "路点 %zu 坐标变换失败，使用原始坐标", i);
            waypoint_poses.push_back(wp.toPose());
        }
    }

    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始笛卡尔路径规划");
    RCLCPP_INFO(logger_, "路点数量: %zu", waypoints.size());
    RCLCPP_INFO(logger_, "最大步长: %.4f m", config.max_step);
    RCLCPP_INFO(logger_, "跳跃阈值: %.3f", config.jump_threshold);
    RCLCPP_INFO(logger_, "避障: %s", config.avoid_collisions ? "是" : "否");
    RCLCPP_INFO(logger_, "速度因子: %.2f", config.max_velocity_factor);
    RCLCPP_INFO(logger_, "=======================================");
    
    // 添加：验证所有输入路点
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        const auto& wp = waypoints[i];
        
        // 打印详细的路径点信息
        RCLCPP_INFO(logger_, "路点 %zu '%s': 位置=[%.6f, %.6f, %.6f]", 
                   i, wp.name.c_str(), wp.x, wp.y, wp.z);
        
        // 检查数据有效性
        if (std::isnan(wp.x) || std::isinf(wp.x) ||
            std::isnan(wp.y) || std::isinf(wp.y) ||
            std::isnan(wp.z) || std::isinf(wp.z)) {
            RCLCPP_ERROR(logger_, "路点 %zu 位置包含无效数据", i);
            RCLCPP_ERROR(logger_, "位置: [%f, %f, %f]", wp.x, wp.y, wp.z);
            return CR7BaseController::Result::INVALID_INPUT;
        }
        
        if (std::isnan(wp.qx) || std::isinf(wp.qx) ||
            std::isnan(wp.qy) || std::isinf(wp.qy) ||
            std::isnan(wp.qz) || std::isinf(wp.qz) ||
            std::isnan(wp.qw) || std::isinf(wp.qw)) {
            RCLCPP_ERROR(logger_, "路点 %zu 四元数包含无效数据", i);
            return CR7BaseController::Result::INVALID_INPUT;
        }
    }
    
    try
    {

        // 获取规划坐标系
        std::string planning_frame = move_group_->getPlanningFrame();
        std::vector<geometry_msgs::msg::Pose> waypoint_poses;

        // 1. 预处理路径点
        if (!preprocessWaypoints(waypoints, waypoint_poses, config))
        {
            RCLCPP_ERROR(logger_, "路径点预处理失败");
            return CR7BaseController::Result::INVALID_INPUT;
        }
        
        // 2. 检查路径可行性
        if (!checkPathFeasibility(waypoint_poses))
        {
            RCLCPP_ERROR(logger_, "路径不可行");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        // 3. 设置规划参数
        move_group_->setMaxVelocityScalingFactor(config.max_velocity_factor);
        
        // 4. 笛卡尔路径规划
        moveit_msgs::msg::RobotTrajectory trajectory;
        RCLCPP_INFO(logger_, "开始笛卡尔路径规划...");
        
        auto start_time = node_->now();

        double fraction = move_group_->computeCartesianPath(
            waypoint_poses, config.eef_step, config.jump_threshold, trajectory, config.avoid_collisions);

        double planning_time = (node_->now() - start_time).seconds();
        
        RCLCPP_INFO(logger_, "笛卡尔规划完成 (耗时 %.3f 秒)", planning_time);
        RCLCPP_INFO(logger_, "路径完成度: %.1f%%", fraction * 100.0);
        
        if (fraction < 0.8)
        {
            RCLCPP_WARN(logger_, "笛卡尔路径完成度较低 (%.1f%%)，尝试优化...", fraction * 100.0);
            
            // 尝试简化路径
            double simplified_fraction = trySimplifiedPath(waypoint_poses, trajectory, config);
            if (simplified_fraction > fraction)
            {
                fraction = simplified_fraction;
                RCLCPP_INFO(logger_, "优化后完成度: %.1f%%", fraction * 100.0);
            }
        }
        
        if (fraction <= 0.0)
        {
            RCLCPP_ERROR(logger_, "笛卡尔路径规划失败");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        // 5. 保存轨迹分析信息
        saveTrajectoryAnalysis(trajectory, "original_trajectory");
        
        // 6. 优化轨迹 - 只做轻微优化，不删除太多点
        if (!optimizeTrajectoryLight(trajectory))
        {
            RCLCPP_WARN(logger_, "轨迹优化失败，使用原始轨迹");
        }
        else
        {
            saveTrajectoryAnalysis(trajectory, "optimized_trajectory");
        }
        
        // 7. 检查轨迹有效性
        if (trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(logger_, "生成的轨迹为空");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "轨迹点数: %zu", trajectory.joint_trajectory.points.size());
        
        // 8. 验证轨迹
        if (!validateTrajectory(trajectory))
        {
            RCLCPP_ERROR(logger_, "轨迹验证失败");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        // 9. 打印轨迹详细信息
        printTrajectoryInfo(trajectory);
        
        // 10. 执行轨迹
        RCLCPP_INFO(logger_, "开始执行笛卡尔路径...");
        start_time = node_->now();
        auto result = move_group_->execute(trajectory);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(logger_, "✓ 笛卡尔路径执行成功 (耗时 %.3f 秒)", execution_time);
            RCLCPP_INFO(logger_, "✓ 完成度: %.1f%%", fraction * 100.0);
            return CR7BaseController::Result::SUCCESS;
        }
        else
        {
            RCLCPP_ERROR(logger_, "笛卡尔路径执行失败 (错误码: %d)", result.val);
            RCLCPP_ERROR(logger_, "耗时: %.3f 秒", execution_time);
            return CR7BaseController::Result::EXECUTION_FAILED;
        }
        
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "笛卡尔路径规划异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

/**
 * @brief 优化的笛卡尔路径规划（推荐使用）
 */
CR7BaseController::Result CR7CartesianPlanner::executeOptimizedCartesianPath(
    const std::vector<Waypoint>& waypoints)
{
    RCLCPP_INFO(logger_, "执行优化的笛卡尔路径规划");
    
    // 使用优化的参数配置
    CartesianPathConfig optimized_config;
    optimized_config.max_step = 0.003;           // 更小的步长提高精度
    optimized_config.eef_step = 0.003;
    optimized_config.max_velocity_factor = 0.2; // 降低速度提高稳定性
    optimized_config.avoid_collisions = true;
    
    return executeCartesianPath(waypoints, optimized_config);
}

// ============================================================================
// 路径预处理和优化方法
// ============================================================================

/**
 * @brief 预处理路径点
 */
bool CR7CartesianPlanner::preprocessWaypoints(
    const std::vector<Waypoint>& input_waypoints,
    std::vector<geometry_msgs::msg::Pose>& output_poses,
    const CartesianPathConfig& config) 
{
    
    output_poses.clear();
    
    if (input_waypoints.empty()) 
    {
        RCLCPP_ERROR(logger_, "输入路径点为空");
        return false;
    }
    
    // 添加起始点（当前位置）
    auto current_pose = getCurrentPose();
    
    // 检查当前位置是否有效
    if (std::isnan(current_pose.position.x) || std::isinf(current_pose.position.x) ||
        std::isnan(current_pose.position.y) || std::isinf(current_pose.position.y) ||
        std::isnan(current_pose.position.z) || std::isinf(current_pose.position.z)) 
    {
        RCLCPP_ERROR(logger_, "当前位置无效，无法作为路径起点");
        RCLCPP_ERROR(logger_, "位置: [%.6f, %.6f, %.6f]", 
                    current_pose.position.x, current_pose.position.y, current_pose.position.z);
        return false;
    }
    
    output_poses.push_back(current_pose);
    RCLCPP_INFO(logger_, "起始点: [%.6f, %.6f, %.6f]", 
                current_pose.position.x, current_pose.position.y, current_pose.position.z);
    
    // 处理输入路径点
    for (size_t i = 0; i < input_waypoints.size(); ++i) {
        const auto& wp = input_waypoints[i];
        
        // 检查输入路点是否有效
        if (std::isnan(wp.x) || std::isinf(wp.x) ||
            std::isnan(wp.y) || std::isinf(wp.y) ||
            std::isnan(wp.z) || std::isinf(wp.z)) {
            RCLCPP_ERROR(logger_, "输入路点 %zu 包含无效数据，跳过", i);
            RCLCPP_ERROR(logger_, "位置: [%f, %f, %f]", wp.x, wp.y, wp.z);
            continue;
        }
        
        // 检查四元数
        if (std::isnan(wp.qx) || std::isinf(wp.qx) ||
            std::isnan(wp.qy) || std::isinf(wp.qy) ||
            std::isnan(wp.qz) || std::isinf(wp.qz) ||
            std::isnan(wp.qw) || std::isinf(wp.qw)) {
            RCLCPP_ERROR(logger_, "输入路点 %zu 四元数包含无效数据，使用单位四元数", i);
            geometry_msgs::msg::Pose pose;
            pose.position.x = wp.x;
            pose.position.y = wp.y;
            pose.position.z = wp.z;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
            output_poses.push_back(pose);
            continue;
        }
        
        // 归一化四元数
        Waypoint normalized_wp = wp;
        if (!wp.isValidQuaternion()) 
        {
            normalized_wp.normalizeQuaternion();
            RCLCPP_DEBUG(logger_, "归一化路点 %zu 的四元数", i);
        }
        
        // 坐标变换：将路点转换到规划坐标系
        geometry_msgs::msg::PoseStamped input_pose = normalized_wp.toPoseStamped();
        geometry_msgs::msg::PoseStamped transformed_pose;
        
        std::string planning_frame = move_group_->getPlanningFrame();
        if (!transformPose(input_pose, transformed_pose, planning_frame)) 
        {
            RCLCPP_ERROR(logger_, "路点 %zu 坐标变换失败", i);
            continue;
        }
        
        geometry_msgs::msg::Pose pose = transformed_pose.pose;
        
        RCLCPP_INFO(logger_, "处理路点 %zu: 位置=[%.6f, %.6f, %.6f]", i,
                    pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(logger_, "  原始坐标系: %s -> 目标坐标系: %s", 
                    wp.frame_id.c_str(), planning_frame.c_str());
        
        // 检查与前一个点的距离
        if (!output_poses.empty()) 
        {
            const auto& prev_pose = output_poses.back();
            double distance = calculateDistance(prev_pose, pose);
            
            RCLCPP_INFO(logger_, "与前一位置距离: %.6fm", distance);
            
            if (distance > 0.3 && distance < 100.0) 
            { // 添加上限检查
                RCLCPP_WARN(logger_, "路径点 %zu 间距过大 (%.3fm)，插入中间点", i, distance);
                if (!insertIntermediatePoints(prev_pose, pose, output_poses, config.max_step)) 
                {
                    RCLCPP_ERROR(logger_, "插入中间点失败");
                    return false;
                }
            } 
            else if (distance < 0.001) 
            { 
                // 如果距离过小，跳过该点
                RCLCPP_WARN(logger_, "路径点 %zu 间距过小 (%.6fm)，跳过", i, distance);
                continue;
            } 
            else if (distance >= 100.0) 
            {
                RCLCPP_ERROR(logger_, "路径点 %zu 间距异常 (%.6fm)，跳过该点", i, distance);
                RCLCPP_ERROR(logger_, "前一点: [%.6f, %.6f, %.6f]", 
                            prev_pose.position.x, prev_pose.position.y, prev_pose.position.z);
                RCLCPP_ERROR(logger_, "当前点: [%.6f, %.6f, %.6f]", 
                            pose.position.x, pose.position.y, pose.position.z);
                continue;
            }
        }
        
        output_poses.push_back(pose);
    }
    
    if (output_poses.size() < 2) 
    {
        RCLCPP_ERROR(logger_, "预处理后路径点数量不足: %zu", output_poses.size());
        return false;
    }
    
    RCLCPP_INFO(logger_, "预处理后路径点数量: %zu", output_poses.size());
    
    // 保存预处理后的路径点用于分析
    saveWaypointAnalysis(output_poses, "preprocessed_waypoints");
    
    return true;
}

/**
 * @brief 检查路径可行性
 */
bool CR7CartesianPlanner::checkPathFeasibility(
    const std::vector<geometry_msgs::msg::Pose>& waypoints) 
{
    
    if (waypoints.size() < 2) 
    {
        RCLCPP_ERROR(logger_, "路径点数量不足");
        return false;
    }
    
    // 首先检查所有路径点的有效性
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        const auto& pose = waypoints[i];
        if (std::isnan(pose.position.x) || std::isinf(pose.position.x) ||
            std::isnan(pose.position.y) || std::isinf(pose.position.y) ||
            std::isnan(pose.position.z) || std::isinf(pose.position.z)) {
            RCLCPP_ERROR(logger_, "路径点 %zu 包含无效的位置数据", i);
            RCLCPP_ERROR(logger_, "位置: [%.6f, %.6f, %.6f]", 
                        pose.position.x, pose.position.y, pose.position.z);
            return false;
        }
    }
    
    // 检查路径点之间的距离
    for (size_t i = 1; i < waypoints.size(); ++i) 
    {
        const auto& prev_pose = waypoints[i-1];
        const auto& curr_pose = waypoints[i];
        
        double distance = calculateDistance(prev_pose, curr_pose);
        if (distance > 100.0) {
            RCLCPP_ERROR(logger_, "路径点 %zu 与前一点的距离异常 (%.3fm)", i, distance);
            return false;
        }
    }
    
    RCLCPP_INFO(logger_, "路径可行性检查通过");
    return true;
}

/**
 * @brief 计算两点间距离
 */
double CR7CartesianPlanner::calculateDistance(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2) const 
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * @brief 插入中间点
 */
bool CR7CartesianPlanner::insertIntermediatePoints(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& end,
    std::vector<geometry_msgs::msg::Pose>& waypoints,
    double max_step) 
{
    
    double distance = calculateDistance(start, end);
    if (distance <= max_step) {
        return true;
    }
    
    int num_points = static_cast<int>(std::ceil(distance / max_step));
    double step = 1.0 / num_points;
    
    RCLCPP_INFO(logger_, "在两点之间插入 %d 个中间点", num_points-1);
    
    for (int i = 1; i < num_points; ++i) 
    {
        double t = i * step;
        
        geometry_msgs::msg::Pose intermediate_pose;
        
        // 线性插值位置
        intermediate_pose.position.x = start.position.x + t * (end.position.x - start.position.x);
        intermediate_pose.position.y = start.position.y + t * (end.position.y - start.position.y);
        intermediate_pose.position.z = start.position.z + t * (end.position.z - start.position.z);
        
        // 球面线性插值四元数
        if (!slerpQuaternion(start.orientation, end.orientation, t, intermediate_pose.orientation)) 
        {
            RCLCPP_ERROR(logger_, "四元数插值失败");
            return false;
        }
        
        waypoints.push_back(intermediate_pose);
    }
    
    return true;
}

/**
 * @brief 球面线性插值四元数
 */
bool CR7CartesianPlanner::slerpQuaternion(
    const geometry_msgs::msg::Quaternion& q1,
    const geometry_msgs::msg::Quaternion& q2,
    double t, geometry_msgs::msg::Quaternion& result) const 
{
    
    double dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
    
    // 确保我们取最短路径
    geometry_msgs::msg::Quaternion q2_used = q2;
    if (dot < 0.0) 
    {
        q2_used.x = -q2_used.x;
        q2_used.y = -q2_used.y;
        q2_used.z = -q2_used.z;
        q2_used.w = -q2_used.w;
        dot = -dot;
    }
    
    // 处理接近的四元数
    if (dot > 0.9995) 
    {
        result.x = q1.x + t * (q2_used.x - q1.x);
        result.y = q1.y + t * (q2_used.y - q1.y);
        result.z = q1.z + t * (q2_used.z - q1.z);
        result.w = q1.w + t * (q2_used.w - q1.w);
        
        // 归一化
        double norm = std::sqrt(result.x*result.x + result.y*result.y + result.z*result.z + result.w*result.w);
        if (norm > 1e-6) 
        {
            result.x /= norm;
            result.y /= norm;
            result.z /= norm;
            result.w /= norm;
        }
        return true;
    }
    
    // 正常的SLERP
    double theta_0 = std::acos(dot);
    double theta = theta_0 * t;
    double sin_theta = std::sin(theta);
    double sin_theta_0 = std::sin(theta_0);
    
    double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    double s1 = sin_theta / sin_theta_0;
    
    result.x = s0 * q1.x + s1 * q2_used.x;
    result.y = s0 * q1.y + s1 * q2_used.y;
    result.z = s0 * q1.z + s1 * q2_used.z;
    result.w = s0 * q1.w + s1 * q2_used.w;
    
    return true;
}

/**
 * @brief 尝试简化路径
 */
double CR7CartesianPlanner::trySimplifiedPath(
    const std::vector<geometry_msgs::msg::Pose>& original_waypoints,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    const CartesianPathConfig& config) 
{
    
    // 简化路径点
    std::vector<geometry_msgs::msg::Pose> simplified_waypoints;
    simplified_waypoints.push_back(original_waypoints[0]);
    
    for (size_t i = 1; i < original_waypoints.size() - 1; ++i) 
    {
        const auto& prev_pose = original_waypoints[i-1];
        const auto& curr_pose = original_waypoints[i];
        const auto& next_pose = original_waypoints[i+1];
        
        double dist_prev_curr = calculateDistance(prev_pose, curr_pose);
        double dist_curr_next = calculateDistance(curr_pose, next_pose);
        double dist_prev_next = calculateDistance(prev_pose, next_pose);
        
        // 如果三点几乎共线，跳过中间点
        if (dist_prev_curr + dist_curr_next < dist_prev_next + 0.001) 
        {
            continue;
        }
        
        simplified_waypoints.push_back(curr_pose);
    }
    
    simplified_waypoints.push_back(original_waypoints.back());
    
    if (simplified_waypoints.size() < 2) 
    {
        return 0.0;
    }
    
    RCLCPP_INFO(logger_, "简化路径后点数量: %zu (原始: %zu)", 
               simplified_waypoints.size(), original_waypoints.size());
    
    // 重新计算笛卡尔路径
    double fraction = move_group_->computeCartesianPath(
        simplified_waypoints, config.eef_step, config.jump_threshold, trajectory, config.avoid_collisions);
    
    return fraction;
}

/**
 * @brief 优化轨迹（轻微优化）
 */
bool CR7CartesianPlanner::optimizeTrajectoryLight(
    moveit_msgs::msg::RobotTrajectory& trajectory) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return false;
    }
    
    const auto& points = trajectory.joint_trajectory.points;
    if (points.size() <= 2) 
    {
        return true; // 点数太少，不需要优化
    }
    
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> optimized_points;
    optimized_points.push_back(points[0]);
    
    for (size_t i = 1; i < points.size() - 1; ++i) 
    {
        if (isPointSignificant(points[i-1], points[i], points[i+1])) 
        {
            optimized_points.push_back(points[i]);
        }
    }
    
    optimized_points.push_back(points.back());
    
    RCLCPP_INFO(logger_, "轨迹优化: %zu -> %zu 点", points.size(), optimized_points.size());
    
    trajectory.joint_trajectory.points = optimized_points;
    return true;
}

/**
 * @brief 检查点是否显著（不应该被删除）
 */
bool CR7CartesianPlanner::isPointSignificant(
    const trajectory_msgs::msg::JointTrajectoryPoint& prev,
    const trajectory_msgs::msg::JointTrajectoryPoint& current,
    const trajectory_msgs::msg::JointTrajectoryPoint& next) const 
{
    
    if (prev.positions.size() != current.positions.size() ||
        current.positions.size() != next.positions.size()) 
    {
        return true;
    }
    
    for (size_t i = 0; i < current.positions.size(); ++i) 
    {
        double delta_prev = std::abs(current.positions[i] - prev.positions[i]);
        double delta_next = std::abs(next.positions[i] - current.positions[i]);
        double delta_prev_next = std::abs(next.positions[i] - prev.positions[i]);
        
        if (delta_prev + delta_next > delta_prev_next + 0.001) 
        {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief 验证轨迹
 */
bool CR7CartesianPlanner::validateTrajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return false;
    }
    
    const auto& points = trajectory.joint_trajectory.points;
    
    // 检查轨迹点的有效性
    for (size_t i = 0; i < points.size(); ++i) 
    {
        const auto& point = points[i];
        
        for (double pos : point.positions) {
            if (std::isnan(pos) || std::isinf(pos)) 
            {
                RCLCPP_ERROR(logger_, "轨迹点 %zu 包含无效的位置数据", i);
                return false;
            }
        }
        
        for (double vel : point.velocities) {
            if (std::isnan(vel) || std::isinf(vel)) 
            {
                RCLCPP_ERROR(logger_, "轨迹点 %zu 包含无效的速度数据", i);
                return false;
            }
        }
    }
    
    // 检查时间戳是否递增
    for (size_t i = 1; i < points.size(); ++i) 
    {
        const auto& prev_time = points[i-1].time_from_start;
        const auto& curr_time = points[i].time_from_start;
        
        if (curr_time.sec < prev_time.sec ||
            (curr_time.sec == prev_time.sec && curr_time.nanosec < prev_time.nanosec)) 
        {
            RCLCPP_ERROR(logger_, "轨迹时间戳不是递增的");
            return false;
        }
    }
    
    RCLCPP_INFO(logger_, "轨迹验证通过");
    return true;
}

/**
 * @brief 保存轨迹分析
 */
void CR7CartesianPlanner::saveTrajectoryAnalysis(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& filename_prefix) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return;
    }
    
    std::string filename = filename_prefix + ".txt";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        RCLCPP_WARN(logger_, "无法创建轨迹分析文件: %s", filename.c_str());
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
    RCLCPP_INFO(logger_, "轨迹分析已保存到: %s", filename.c_str());
}

/**
 * @brief 保存路径点分析
 */
void CR7CartesianPlanner::saveWaypointAnalysis(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const std::string& filename_prefix) 
{
    
    if (waypoints.empty()) 
    {
        return;
    }
    
    std::string filename = filename_prefix + ".txt";
    std::ofstream file(filename);
    
    if (!file.is_open()) 
    {
        RCLCPP_WARN(logger_, "无法创建路径点分析文件: %s", filename.c_str());
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
    RCLCPP_INFO(logger_, "路径点分析已保存到: %s", filename.c_str());
}

/**
 * @brief 打印轨迹信息
 */
void CR7CartesianPlanner::printTrajectoryInfo(
    const moveit_msgs::msg::RobotTrajectory& trajectory) 
{
    
    if (trajectory.joint_trajectory.points.empty()) 
    {
        return;
    }
    
    const auto& points = trajectory.joint_trajectory.points;
    
    RCLCPP_INFO(logger_, "轨迹详细信息:");
    RCLCPP_INFO(logger_, "- 轨迹点数: %zu", points.size());
    
    // 打印轨迹的起始和结束时间
    double start_time = points[0].time_from_start.sec + 
                       points[0].time_from_start.nanosec * 1e-9;
    double end_time = points.back().time_from_start.sec + 
                     points.back().time_from_start.nanosec * 1e-9;
    
    RCLCPP_INFO(logger_, "- 起始时间: %.3f 秒", start_time);
    RCLCPP_INFO(logger_, "- 结束时间: %.3f 秒", end_time);
    RCLCPP_INFO(logger_, "- 总时间: %.3f 秒", end_time - start_time);
}

/**
 * @brief 获取当前机器人末端执行器位姿
 */
geometry_msgs::msg::Pose CR7CartesianPlanner::getCurrentPose() 
{
    return CR7BaseController::getCurrentPose();
}

} // namespace cr7_controller
