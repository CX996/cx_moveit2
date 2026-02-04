/**
 * @file cr7_robot_controller.cpp
 * @brief CR7机器人控制器主实现文件
 * 
 * 实现CR7RobotController类的所有方法
 * 作为协调器整合所有模块
 */
#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <fstream>

#include "cr7_robot_controller/cr7_robot_controller.hpp"

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7RobotController 构造函数
// ============================================================================

CR7RobotController::CR7RobotController(rclcpp::Node::SharedPtr node, 
                                      const std::string& planning_group)
    : CR7BaseController(node, planning_group)
    , cartesian_planner_(nullptr)
    , pilz_planner_(nullptr)
    , path_executor_(nullptr)
{
    RCLCPP_INFO(logger_, "创建CR7机器人控制器");
    RCLCPP_INFO(logger_, "规划组: %s", planning_group.c_str());

    try
    {
        // 初始化笛卡尔路径规划器
        cartesian_planner_ = std::make_shared<CR7CartesianPlanner>(node_, move_group_, logger_);
        RCLCPP_INFO(logger_, "笛卡尔路径规划器初始化成功");
        
        // 初始化PILZ工业规划器
        pilz_planner_ = std::make_shared<CR7PilzPlanner>(node_, move_group_, logger_);
        RCLCPP_INFO(logger_, "PILZ工业规划器初始化成功");
        
        // 初始化预设路径执行器
        path_executor_ = std::make_shared<CR7PathExecutor>(node_, planning_group, cartesian_planner_, pilz_planner_);
        RCLCPP_INFO(logger_, "预设路径执行器初始化成功");
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger_, "初始化模块失败: %s", e.what());
        throw;
    }
}

// ============================================================================
// 模块整合方法
// ============================================================================

// 笛卡尔路径规划方法 - 委托给笛卡尔规划器模块
CR7RobotController::Result CR7RobotController::executeCartesianPath(
    const std::vector<Waypoint>& waypoints,
    const CartesianPathConfig& config)
{
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!cartesian_planner_) {
        RCLCPP_ERROR(logger_, "笛卡尔路径规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return cartesian_planner_->executeCartesianPath(waypoints, config);
}

// 优化的笛卡尔路径规划方法
CR7RobotController::Result CR7RobotController::executeOptimizedCartesianPath(
    const std::vector<Waypoint>& waypoints)
{
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!cartesian_planner_) {
        RCLCPP_ERROR(logger_, "笛卡尔路径规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return cartesian_planner_->executeOptimizedCartesianPath(waypoints);
}

/**
 * @brief 测试PILZ规划器功能
 * @param planner_type 规划器类型
 * @return Result 测试结果
 */
CR7RobotController::Result testPilzPlanner(PilzPlanner planner_type)
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!pilz_planner_) {
        RCLCPP_ERROR(logger_, "PILZ规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }

    return pilz_planner_->testPilzPlanner(target_pose, config);
}
    
// PILZ工业规划器方法 - 委托给PILZ规划器模块
CR7RobotController::Result CR7RobotController::moveWithPilzLin(const geometry_msgs::msg::Pose& target_pose,
                                                              const PilzConfig& config)
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!pilz_planner_) {
        RCLCPP_ERROR(logger_, "PILZ规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return pilz_planner_->moveWithPilzLin(target_pose, config);
}

CR7RobotController::Result CR7RobotController::moveWithPilzPtp(const geometry_msgs::msg::Pose& target_pose,
                                                              const PilzConfig& config)
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!pilz_planner_) {
        RCLCPP_ERROR(logger_, "PILZ规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return pilz_planner_->moveWithPilzPtp(target_pose, config);
}

CR7RobotController::Result CR7RobotController::moveWithPilzCirc(const geometry_msgs::msg::Pose& intermediate_pose,
                                                              const geometry_msgs::msg::Pose& target_pose,
                                                              const PilzConfig& config)
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!pilz_planner_) {
        RCLCPP_ERROR(logger_, "PILZ规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return pilz_planner_->moveWithPilzCirc(intermediate_pose, target_pose, config);
}

std::vector<CR7RobotController::Result> CR7RobotController::executeWeldingPath()
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return {Result::ROBOT_NOT_READY};
    }
    
    if (!path_executor_) {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return {Result::ROBOT_NOT_READY};
    }
    
    return path_executor_->executeWeldingPath();
}

std::vector<CR7RobotController::Result> CR7RobotController::executeTestPath()
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return {Result::ROBOT_NOT_READY};
    }
    
    if (!path_executor_) {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return {Result::ROBOT_NOT_READY};
    }
    
    return path_executor_->executeTestPath();
}

CR7RobotController::Result CR7RobotController::executeCartesianWeldingPath()
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!path_executor_) {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return path_executor_->executeCartesianWeldingPath();
}

CR7RobotController::Result CR7RobotController::executePilzWeldingPath()
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!path_executor_) 
    {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return path_executor_->executePilzWeldingPath();
}

CR7RobotController::Result CR7RobotController::executeToolAxis()
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!path_executor_) 
    {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return path_executor_->executeToolAxis();
}

// ============================================================================
// 状态查询和配置方法
// ============================================================================

bool CR7RobotController::isReady() const
{
    return initialized_;
}

geometry_msgs::msg::Pose CR7RobotController::getCurrentPose() const
{
    if (!initialized_) 
    {
        RCLCPP_WARN(logger_, "控制器未初始化");
        return geometry_msgs::msg::Pose();
    }
    
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

std::vector<double> CR7RobotController::getCurrentJointPositions() const
{
    if (!initialized_) 
    {
        RCLCPP_WARN(logger_, "控制器未初始化");
        return {};
    }
    
    try 
    {
        return move_group_->getCurrentJointValues();
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "获取关节位置失败: %s", e.what());
        return {};
    }
}

void CR7RobotController::printCurrentState() const
{
    if (!initialized_) 
    {
        RCLCPP_INFO(logger_, "控制器状态: 未初始化");
        return;
    }
    
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "当前机器人状态");
    RCLCPP_INFO(logger_, "=======================================");
    
    try 
    {
        // 获取末端位姿
        auto pose = getCurrentPose();
        RCLCPP_INFO(logger_, "末端位姿:");
        RCLCPP_INFO(logger_, "  位置: [%.3f, %.3f, %.3f]", 
                   pose.position.x, pose.position.y, pose.position.z);
        RCLCPP_INFO(logger_, "  姿态: [%.3f, %.3f, %.3f, %.3f]",
                   pose.orientation.x, pose.orientation.y,
                   pose.orientation.z, pose.orientation.w);
        
        // 获取关节位置
        auto joints = getCurrentJointPositions();
        if (!joints.empty()) 
        {
            RCLCPP_INFO(logger_, "关节位置 (弧度):");
            for (size_t i = 0; i < joints.size(); ++i) 
            {
                RCLCPP_INFO(logger_, "  关节%zu: %.4f", i + 1, joints[i]);
            }
        }
        
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "获取状态失败: %s", e.what());
    }
    
    RCLCPP_INFO(logger_, "=======================================");
}

void CR7RobotController::setPlanningTime(double seconds)
{
    if (seconds > 0) 
    {
        move_group_->setPlanningTime(seconds);
        RCLCPP_INFO(logger_, "设置规划时间: %.1f 秒", seconds);
    }
}

void CR7RobotController::setVelocityFactor(double factor)
{
    if (factor > 0 && factor <= 1.0) 
    {
        move_group_->setMaxVelocityScalingFactor(factor);
        RCLCPP_INFO(logger_, "设置速度因子: %.2f", factor);
    }
}

void CR7RobotController::setAccelerationFactor(double factor)
{
    if (factor > 0 && factor <= 1.0) 
    {
        move_group_->setMaxAccelerationScalingFactor(factor);
        RCLCPP_INFO(logger_, "设置加速度因子: %.2f", factor);
    }
}

}  // namespace cr7_controller
