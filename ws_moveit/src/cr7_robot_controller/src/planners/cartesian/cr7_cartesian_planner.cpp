/**
 * @file cr7_cartesian_planner.cpp
 * @brief 笛卡尔路径规划模块实现文件
 * 
 * 实现CR7CartesianPlanner类的所有方法
 */

#include <cmath>
#include <chrono>

#include "cr7_robot_controller/planners/cartesian/cr7_cartesian_planner.hpp"
#include "cr7_robot_controller/utils/trajectory_analyzer.hpp"
#include "cr7_robot_controller/utils/pose_utils.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7CartesianPlanner 构造函数
// ============================================================================
/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param move_group MoveGroup接口指针
 * @param logger 日志记录器
 */
CR7CartesianPlanner::CR7CartesianPlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group
)   : node_(node), move_group_(move_group), logger_(node->get_logger()), config_()
{
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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
    
    RCLCPP_INFO(logger_, "开始笛卡尔路径规划");
    RCLCPP_INFO(logger_, "路点数量: %zu", waypoints.size());
    
    try
    {
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
        
        if (fraction <= 0.0)
        {
            RCLCPP_ERROR(logger_, "笛卡尔路径规划失败");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        // 5. 保存轨迹分析信息
        utils::TrajectoryAnalyzer::saveTrajectoryAnalysis(trajectory, "original_trajectory", logger_);
        
        // 6. 检查轨迹有效性
        if (trajectory.joint_trajectory.points.empty())
        {
            RCLCPP_ERROR(logger_, "生成的轨迹为空");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "轨迹点数: %zu", trajectory.joint_trajectory.points.size());
        
        // 7. 验证轨迹
        if (!validateTrajectory(trajectory))
        {
            RCLCPP_ERROR(logger_, "轨迹验证失败");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        // 8. 打印轨迹详细信息
        utils::TrajectoryAnalyzer::printTrajectoryInfo(trajectory, logger_);
        
        // 9. 执行轨迹
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
        if (!utils::PoseUtils::transformPose(input_pose, transformed_pose, planning_frame, tf_buffer_, 2.0, logger_)) 
        {
            RCLCPP_ERROR(logger_, "路点 %zu 坐标变换失败", i);
            continue;
        }
        
        geometry_msgs::msg::Pose pose = transformed_pose.pose;
        
        RCLCPP_INFO(logger_, "处理路点 %zu: 位置=[%.6f, %.6f, %.6f]", i,
                    pose.position.x, pose.position.y, pose.position.z);
        
        // 检查与前一个点的距离
        if (!output_poses.empty()) 
        {
            const auto& prev_pose = output_poses.back();
            double dx = prev_pose.position.x - pose.position.x;
            double dy = prev_pose.position.y - pose.position.y;
            double dz = prev_pose.position.z - pose.position.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < 0.001) 
            {
                // 如果距离过小，跳过该点
                RCLCPP_WARN(logger_, "路径点 %zu 间距过小 (%.6fm)，跳过", i, distance);
                continue;
            } 
            else if (distance >= 100.0)
            {
                RCLCPP_ERROR(logger_, "路径点 %zu 间距异常 (%.6fm)，跳过该点", i, distance);
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
    utils::TrajectoryAnalyzer::saveWaypointAnalysis(output_poses, "preprocessed_waypoints", logger_);
    
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
            return false;
        }
    }
    
    // 检查路径点之间的距离
    for (size_t i = 1; i < waypoints.size(); ++i) 
    {
        const auto& prev_pose = waypoints[i-1];
        const auto& curr_pose = waypoints[i];
        
        double dx = prev_pose.position.x - curr_pose.position.x;
        double dy = prev_pose.position.y - curr_pose.position.y;
        double dz = prev_pose.position.z - curr_pose.position.z;
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (distance > 100.0) {
            RCLCPP_ERROR(logger_, "路径点 %zu 与前一点的距离异常 (%.3fm)", i, distance);
            return false;
        }
    }
    
    RCLCPP_INFO(logger_, "路径可行性检查通过");
    return true;
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
 * @brief 获取当前机器人末端执行器位姿
 */
geometry_msgs::msg::Pose CR7CartesianPlanner::getCurrentPose() 
{
    try 
    {
        return move_group_->getCurrentPose().pose;
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "获取当前位姿失败: %s", e.what());
        return geometry_msgs::msg::Pose();
    }
}

} // namespace cr7_controller
