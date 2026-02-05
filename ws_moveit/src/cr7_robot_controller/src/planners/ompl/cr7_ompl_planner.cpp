/**
 * @file cr7_ompl_planner.cpp
 * @brief OMPL规划器模块实现文件
 * 
 * 实现CR7OMPLPlanner类的所有方法
 */

#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "cr7_robot_controller/planners/ompl/cr7_ompl_planner.hpp"
#include "cr7_robot_controller/utils/pose_utils.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7OMPLPlanner 方法实现
// ============================================================================

/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param move_group MoveGroup接口指针
 * @param logger 日志记录器
 */
CR7OMPLPlanner::CR7OMPLPlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group
)   : node_(node)
    , move_group_(move_group)
    , logger_(node->get_logger())
{
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(logger_, "创建OMPL规划器");
    RCLCPP_INFO(logger_, "默认规划器: %s", config_.planner_id.c_str());

    // 设置默认规划参数
    move_group_->setPlannerId(config_.planner_id);
    move_group_->setPlanningTime(config_.planning_time);
    move_group_->setNumPlanningAttempts(config_.num_planning_attempts);
    move_group_->setMaxVelocityScalingFactor(config_.velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(config_.acceleration_scale);
    move_group_->setGoalPositionTolerance(config_.goal_position_tolerance);
    move_group_->setGoalOrientationTolerance(config_.goal_orientation_tolerance);

    RCLCPP_INFO(logger_, "OMPL规划器初始化完成");
}

/**
 * @brief 移动到单个位姿
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPose(const geometry_msgs::msg::Pose& target_pose,
                                                   const std::string& waypoint_name) 
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = move_group_->getPlanningFrame(); // 默认规划坐标系
    pose_stamped.pose = target_pose;
    return moveToPose(pose_stamped, waypoint_name);
}

/**
 * @brief 移动到单个位姿，支持PoseStamped
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPose(const geometry_msgs::msg::PoseStamped& target_pose,
                                                   const std::string& waypoint_name) 
{
    // 获取规划坐标系（通常是机器人的基坐标系）
    std::string planning_frame = move_group_->getPlanningFrame();
    
    geometry_msgs::msg::PoseStamped transformed_pose;
    
    // 如果目标坐标系与规划坐标系不同，进行坐标变换
    if (target_pose.header.frame_id != planning_frame) {
        RCLCPP_INFO(logger_, "变换坐标系: %s -> %s", 
                    target_pose.header.frame_id.c_str(), planning_frame.c_str());
        
        if (!utils::PoseUtils::transformPose(target_pose, transformed_pose, planning_frame, tf_buffer_, 2.0, logger_)) {
            RCLCPP_ERROR(logger_, "坐标变换失败");
            return CR7BaseController::Result::INVALID_INPUT;
        }
    } else {
        transformed_pose = target_pose;
    }
    
    return moveToPoseImpl(transformed_pose.pose, waypoint_name);
}

/**
 * @brief 执行多路点序列
 * @param waypoints 路点向量
 * @param delay_seconds 路点间延迟(秒)
 * @return std::vector<CR7BaseController::Result> 每个路点的结果
 */
std::vector<CR7BaseController::Result> CR7OMPLPlanner::executeWaypoints(
    const std::vector<Waypoint>& waypoints,
    double delay_seconds) 
{
    std::vector<CR7BaseController::Result> results;
    
    RCLCPP_INFO(logger_, "开始执行 %zu 个路点，路点间延迟: %.1f 秒", waypoints.size(), delay_seconds);
    
    int success_count = 0;
    
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        const auto& wp = waypoints[i];
        
        RCLCPP_INFO(logger_, "执行路点 %zu/%zu: %s", i + 1, waypoints.size(), wp.name.c_str());
        
        // 执行路点
        auto result = moveToPose(wp.toPoseStamped(), wp.name);
        results.push_back(result);
        
        if (result == CR7BaseController::Result::SUCCESS) 
        {
            success_count++;
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "路点 %s 失败: %s", 
                        wp.name.c_str(), utils::PoseUtils::resultToString(result).c_str());
        }
        
        // 路点间延迟（最后一个路点后不延迟）
        if (i < waypoints.size() - 1 && delay_seconds > 0) 
        {
            RCLCPP_INFO(logger_, "等待 %.1f 秒...", delay_seconds);
            rclcpp::sleep_for(std::chrono::duration<double>(delay_seconds));
        }
    }
    
    RCLCPP_INFO(logger_, "路点序列执行完成，成功: %d/%zu", success_count, waypoints.size());
    
    return results;
}

/**
 * @brief 设置规划时间
 * @param seconds 规划时间(秒)
 */
void CR7OMPLPlanner::setPlanningTime(double seconds) 
{
    config_.planning_time = seconds;
    move_group_->setPlanningTime(seconds);
    RCLCPP_INFO(logger_, "设置规划时间: %.1f秒", seconds);
}

/**
 * @brief 设置速度因子
 * @param factor 速度因子(0.0-1.0)
 */
void CR7OMPLPlanner::setVelocityFactor(double factor) 
{
    if (factor < 0.0 || factor > 1.0) 
    {
        RCLCPP_ERROR(logger_, "速度因子必须在0.0-1.0之间");
        return;
    }
    
    config_.velocity_scale = factor;
    move_group_->setMaxVelocityScalingFactor(factor);
    RCLCPP_INFO(logger_, "设置速度因子: %.2f", factor);
}

/**
 * @brief 设置加速度因子
 * @param factor 加速度因子(0.0-1.0)
 */
void CR7OMPLPlanner::setAccelerationFactor(double factor) 
{
    if (factor < 0.0 || factor > 1.0) 
    {
        RCLCPP_ERROR(logger_, "加速度因子必须在0.0-1.0之间");
        return;
    }
    
    config_.acceleration_scale = factor;
    move_group_->setMaxAccelerationScalingFactor(factor);
    RCLCPP_INFO(logger_, "设置加速度因子: %.2f", factor);
}

/**
 * @brief 内部实现移动到位姿
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return CR7BaseController::Result 规划结果
 */                                     
CR7BaseController::Result CR7OMPLPlanner::moveToPoseImpl(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& waypoint_name) 
{
    // 验证输入
    std::string error_msg;
    if (!utils::PoseUtils::validatePose(target_pose, &error_msg)) {
        RCLCPP_ERROR(logger_, "无效的位姿: %s", error_msg.c_str());
        return CR7BaseController::Result::INVALID_INPUT;
    }
    
    // 打印目标信息
    if (!waypoint_name.empty()) {
        RCLCPP_INFO(logger_, "移动到路点: %s", waypoint_name.c_str());
    }
    RCLCPP_INFO(logger_, "目标位置: [%.3f, %.3f, %.3f]", 
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(logger_, "目标姿态: [%.3f, %.3f, %.3f, %.3f]",
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w);
    
    try 
    {
        // 设置目标位姿
        move_group_->setPoseTarget(target_pose);
        
        // 规划运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO(logger_, "开始规划...");
        
        auto start_time = node_->now();
        bool success = static_cast<bool>(move_group_->plan(plan));
        double planning_time = (node_->now() - start_time).seconds();
        
        if (!success || plan.trajectory_.joint_trajectory.points.empty()) 
        {
            RCLCPP_ERROR(logger_, "规划失败 (耗时 %.3f 秒)", planning_time);
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "规划成功 (耗时 %.3f 秒), 轨迹点数: %zu", planning_time, plan.trajectory_.joint_trajectory.points.size());
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行...");
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) 
        {
            RCLCPP_INFO(logger_, "执行成功 (耗时 %.3f 秒)", execution_time);
            if (!waypoint_name.empty()) {
                RCLCPP_INFO(logger_, "到达路点: %s", waypoint_name.c_str());
            }
            return CR7BaseController::Result::SUCCESS;
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "执行失败 (错误码: %d, 耗时: %.3f 秒)", result.val, execution_time);
            return CR7BaseController::Result::EXECUTION_FAILED;
        }
        
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "运动控制异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

}  // namespace cr7_controller
