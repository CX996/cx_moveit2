/**
 * @file cr7_robot_controller.cpp
 * @brief CR7机器人控制器主实现文件
 * 
 * 实现CR7RobotController类的所有方法
 * 作为协调器整合所有模块
 */
#include <cmath>
#include <chrono>

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
    , ompl_planner_(nullptr)
    , path_executor_(nullptr)
{
    RCLCPP_INFO(logger_, "创建CR7机器人控制器");
    RCLCPP_INFO(logger_, "规划组: %s", planning_group.c_str());

    try
    {
        // 初始化笛卡尔路径规划器
        cartesian_planner_ = std::make_shared<CR7CartesianPlanner>(node_, move_group_);
        RCLCPP_INFO(logger_, "笛卡尔路径规划器初始化成功");
        
        // 初始化PILZ工业规划器
        pilz_planner_ = std::make_shared<CR7PilzPlanner>(node_, move_group_);
        RCLCPP_INFO(logger_, "PILZ工业规划器初始化成功");
        
        // 初始化OMPL规划器
        ompl_planner_ = std::make_shared<CR7OMPLPlanner>(node_, move_group_);
        RCLCPP_INFO(logger_, "OMPL规划器初始化成功");
        
        // 初始化测试路径执行器
        path_executor_ = std::make_shared<CR7PathExecutor>(node_, move_group_, cartesian_planner_, pilz_planner_, ompl_planner_);
        RCLCPP_INFO(logger_, "测试路径执行器初始化成功");
        
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

// 测试路径执行方法 - 委托给路径执行器模块
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

CR7RobotController::Result CR7RobotController::executeCartesianTestPath()
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!path_executor_) {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return path_executor_->executeCartesianTestPath();
}

CR7RobotController::Result CR7RobotController::executePilzTestPath()
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!path_executor_) {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return path_executor_->executePilzTestPath();
}

CR7RobotController::Result CR7RobotController::executeToolAxisTestPath()
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!path_executor_) {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return path_executor_->executeToolAxisTestPath();
}

CR7RobotController::Result CR7RobotController::executeWeldingTestPath()
{
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!path_executor_) {
        RCLCPP_ERROR(logger_, "路径执行器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return path_executor_->executeWeldingTestPath();
}

// ============================================================================
// OMPL约束规划方法实现
// ============================================================================

void CR7RobotController::setPositionConstraintBox(
    const std::string& link_name,
    double min_x, double max_x,
    double min_y, double max_y,
    double min_z, double max_z,
    const std::string& frame_id
) {
    if (!ompl_planner_) {
        RCLCPP_ERROR(logger_, "OMPL规划器未初始化");
        return;
    }
    
    ompl_planner_->setPositionConstraintBox(link_name, min_x, max_x, min_y, max_y, min_z, max_z, frame_id);
}

void CR7RobotController::setPositionConstraintPlane(
    const std::string& link_name,
    const geometry_msgs::msg::Vector3& plane_normal,
    double distance,
    const std::string& frame_id
) {
    if (!ompl_planner_) {
        RCLCPP_ERROR(logger_, "OMPL规划器未初始化");
        return;
    }
    
    ompl_planner_->setPositionConstraintPlane(link_name, plane_normal, distance, frame_id);
}

void CR7RobotController::setPositionConstraintLine(
    const std::string& link_name,
    const geometry_msgs::msg::Point& line_start,
    const geometry_msgs::msg::Point& line_end,
    const std::string& frame_id
) {
    if (!ompl_planner_) {
        RCLCPP_ERROR(logger_, "OMPL规划器未初始化");
        return;
    }
    
    ompl_planner_->setPositionConstraintLine(link_name, line_start, line_end, frame_id);
}

void CR7RobotController::setOrientationConstraint(
    const std::string& link_name,
    const geometry_msgs::msg::Quaternion& orientation,
    double tolerance_x,
    double tolerance_y,
    double tolerance_z,
    const std::string& frame_id
) {
    if (!ompl_planner_) {
        RCLCPP_ERROR(logger_, "OMPL规划器未初始化");
        return;
    }
    
    ompl_planner_->setOrientationConstraint(link_name, orientation, tolerance_x, tolerance_y, tolerance_z, frame_id);
}

void CR7RobotController::clearConstraints() {
    if (!ompl_planner_) {
        RCLCPP_ERROR(logger_, "OMPL规划器未初始化");
        return;
    }
    
    ompl_planner_->clearConstraints();
}

CR7RobotController::Result CR7RobotController::moveToPoseWithConstraints(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& waypoint_name
) {
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (!ompl_planner_) {
        RCLCPP_ERROR(logger_, "OMPL规划器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    return ompl_planner_->moveToPoseWithConstraints(target_pose, waypoint_name);
}

} // namespace cr7_controller
