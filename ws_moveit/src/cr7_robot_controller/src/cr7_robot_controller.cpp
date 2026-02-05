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
        cartesian_planner_ = std::make_shared<CR7CartesianPlanner>(node_, planning_group, logger_);
        RCLCPP_INFO(logger_, "笛卡尔路径规划器初始化成功");
        
        // 初始化PILZ工业规划器
        pilz_planner_ = std::make_shared<CR7PilzPlanner>(node_, move_group_, logger_);
        RCLCPP_INFO(logger_, "PILZ工业规划器初始化成功");
        
        // 初始化预设路径执行器
        path_executor_ = std::make_shared<CR7PathExecutor>(node_, planning_group, move_group_, cartesian_planner_, pilz_planner_);
        RCLCPP_INFO(logger_, "预设路径执行器初始化成功");
        
    } 
    catch (const std::exception& e) 
    {
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
CR7RobotController::Result CR7RobotController::testPilzPlanner(PilzPlanner planner_type)
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!pilz_planner_) {
        RCLCPP_ERROR(logger_, "PILZ规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }

    return pilz_planner_->testPilzPlanner(planner_type);
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
}
 // namespace cr7_controller
