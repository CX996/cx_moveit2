/**
 * @file cr7_path_executor.cpp
 * @brief 预设路径执行模块实现文件
 * 
 * 实现CR7PathExecutor类的所有方法
 */

#include <cmath>
#include <chrono>

#include "cr7_robot_controller/path/cr7_path_executor.hpp"

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7PathExecutor 构造函数
// ============================================================================

/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param planning_group MoveIt规划组名称
 * @param cartesian_planner 笛卡尔路径规划器指针
 * @param pilz_planner PILZ规划器指针
 */
CR7PathExecutor::CR7PathExecutor(
    rclcpp::Node::SharedPtr node,
    const std::string& planning_group,
    std::shared_ptr<CR7CartesianPlanner> cartesian_planner,
    std::shared_ptr<CR7PilzPlanner> pilz_planner
) : CR7BaseController(node, planning_group), cartesian_planner_(cartesian_planner), pilz_planner_(pilz_planner)
{
    RCLCPP_INFO(logger_, "创建预设路径执行器");
}

// ============================================================================
// 预设路径执行方法
// ============================================================================

/**
 * @brief 执行预设焊接路径
 * @return std::vector<Result> 每个路点的结果
 */
std::vector<CR7BaseController::Result> CR7PathExecutor::executeWeldingPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行预设焊接路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    // 创建焊接路径点
    std::vector<Waypoint> waypoints = generateWeldingPath();
    
    if (waypoints.empty())
    {
        RCLCPP_ERROR(logger_, "创建焊接路径失败");
        return {CR7BaseController::Result::INVALID_INPUT};
    }
    
    RCLCPP_INFO(logger_, "焊接路径点数量: %zu", waypoints.size());
    
    // 验证四元数
    for (auto& wp : waypoints)
    {
        if (!wp.isValidQuaternion())
        {
            RCLCPP_WARN(logger_, "路点 %s 四元数未归一化，自动处理", wp.name.c_str());
            wp.normalizeQuaternion();
        }
    }
    
    // 使用父类的 executeWaypoints 方法执行路径
    return this->executeWaypoints(waypoints, 2.0);
}

/**
 * @brief 执行测试路径
 * @return std::vector<Result> 每个路点的结果
 */
std::vector<CR7BaseController::Result> CR7PathExecutor::executeTestPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行测试路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    // 创建测试路径点
    std::vector<Waypoint> waypoints = generateTestPath();
    
    if (waypoints.empty())
    {
        RCLCPP_ERROR(logger_, "创建测试路径失败");
        return {CR7BaseController::Result::INVALID_INPUT};
    }
    
    RCLCPP_INFO(logger_, "测试路径点数量: %zu", waypoints.size());
    
    // 验证四元数
    for (auto& wp : waypoints)
    {
        if (!wp.isValidQuaternion())
        {
            RCLCPP_WARN(logger_, "路点 %s 四元数未归一化，自动处理", wp.name.c_str());
            wp.normalizeQuaternion();
        }
    }
    
    // 使用父类的 executeWaypoints 方法执行路径
    return this->executeWaypoints(waypoints, 2.0);
}

/**
 * @brief 执行笛卡尔焊接路径
 * @return Result 规划结果
 */
CR7BaseController::Result CR7PathExecutor::executeCartesianWeldingPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行笛卡尔焊接路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    // 创建笛卡尔焊接路径点
    std::vector<Waypoint> waypoints = generateCartesianWeldingPath();
    
    if (waypoints.empty())
    {
        RCLCPP_ERROR(logger_, "创建笛卡尔焊接路径失败");
        return CR7BaseController::Result::INVALID_INPUT;
    }
    
    RCLCPP_INFO(logger_, "笛卡尔焊接路径点数量: %zu", waypoints.size());
    
    // 使用笛卡尔路径规划器执行路径
    if (cartesian_planner_)
    {
        return cartesian_planner_->executeOptimizedCartesianPath(waypoints);
    }
    else
    {
        RCLCPP_ERROR(logger_, "笛卡尔路径规划器未初始化");
        return CR7BaseController::Result::ROBOT_NOT_READY;
    }
}

/**
 * @brief 使用PILZ执行焊接点位路径
 * 先用关节空间移动到起点，再用PILZ执行到终点
 * @return Result 规划结果
 */
CR7BaseController::Result CR7PathExecutor::executePilzWeldingPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行PILZ焊接路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    // 这里我们创建一个简单的焊接路径，使用PILZ LIN规划器执行
    // 实际应用中，应该根据具体的焊接任务创建更复杂的路径

    // 创建起点和终点
    Waypoint start_waypoint("welding_start", 0.4, 0.0, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    this->moveToWaypoint(start_waypoint);  // 先移动到起点

    Waypoint end_waypoint("welding_end", 0.55, 0.15, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");

    // 使用PILZ LIN规划器执行直线运动
    if (pilz_planner_)
    {
        return pilz_planner_->moveWithPilzLin(end_waypoint.toPose());
    }
    else
    {
        RCLCPP_ERROR(logger_, "PILZ规划器未初始化");
        return CR7BaseController::Result::ROBOT_NOT_READY;
    }
}

/**
 * @brief 使用工具坐标系进行点位执行
 * @return Result 规划结果
 */
CR7BaseController::Result CR7PathExecutor::executeToolAxis()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行工具坐标系路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    // 创建工具坐标系路径点
    std::vector<Waypoint> waypoints = generateToolAxisPath();
    
    if (waypoints.empty())
    {
        RCLCPP_ERROR(logger_, "创建工具坐标系路径失败");
        return CR7BaseController::Result::INVALID_INPUT;
    }
    
    RCLCPP_INFO(logger_, "工具坐标系路径点数量: %zu", waypoints.size());
    
    // 使用笛卡尔路径规划器执行路径
    if (cartesian_planner_)
    {
        return cartesian_planner_->executeOptimizedCartesianPath(waypoints);
    }
    else
    {
        RCLCPP_ERROR(logger_, "笛卡尔路径规划器未初始化");
        return CR7BaseController::Result::ROBOT_NOT_READY;
    }
}

// ============================================================================
// 路径创建方法
// ============================================================================
/**
 * @brief 生成焊接路径点
 * @return std::vector<Waypoint> 焊接路径点
 */
std::vector<Waypoint> CR7PathExecutor::generateWeldingPath()
{
    std::vector<Waypoint> waypoints;
    
    // 创建焊接路径点
    // 这里创建一个简单的焊接路径，实际应用中应该根据具体的焊接任务创建更复杂的路径
    
    // 起点
    Waypoint start("welding_start", 0.4, 0.0, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(start);
    
    // 中间点1
    Waypoint mid1("welding_mid1", 0.45, 0.05, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(mid1);
    
    // 中间点2
    Waypoint mid2("welding_mid2", 0.5, 0.1, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(mid2);
    
    // 终点
    Waypoint end("welding_end", 0.55, 0.15, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(end);
    
    return waypoints;
}

/**
 * @brief 生成测试路径点
 * @return std::vector<Waypoint> 测试路径点
 */
std::vector<Waypoint> CR7PathExecutor::generateTestPath()
{
    std::vector<Waypoint> waypoints;
    
    // 创建测试路径点
    // 这里创建一个简单的测试路径，用于验证机器人的基本运动能力
    
    // 起点
    Waypoint start("test_start", 0.3, -0.2, 0.4, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(start);
    
    // 点1
    Waypoint point1("test_point1", 0.4, 0.0, 0.4, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(point1);
    
    // 点2
    Waypoint point2("test_point2", 0.3, 0.2, 0.4, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(point2);
    
    // 点3
    Waypoint point3("test_point3", 0.2, 0.0, 0.4, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(point3);
    
    // 回到起点
    waypoints.push_back(start);
    
    return waypoints;
}

/**
 * @brief 生成笛卡尔焊接路径点
 * @return std::vector<Waypoint> 笛卡尔路径点
 */
std::vector<Waypoint> CR7PathExecutor::generateCartesianWeldingPath()
{
    std::vector<Waypoint> waypoints;
    
    // 创建笛卡尔焊接路径点
    // 这里创建一个简单的直线焊接路径，实际应用中应该根据具体的焊接任务创建更复杂的路径
    
    // 起点
    Waypoint start("cartesian_welding_start", 0.4, -0.1, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(start);
    
    // 终点
    Waypoint end("cartesian_welding_end", 0.6, 0.1, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(end);
    
    return waypoints;
}

    /**
     * @brief 生成工具坐标系路径点
     * @return std::vector<Waypoint> 工具坐标系路径点
     */
std::vector<Waypoint> CR7PathExecutor::generateToolAxisPath()
{
    std::vector<Waypoint> waypoints;
    
    // 创建工具坐标系路径点
    // 这里创建一个简单的工具坐标系路径，用于验证工具坐标系的运动能力
    
    // 起点
    Waypoint start("tool_axis_start", 0.4, 0.0, 0.4, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(start);
    
    // 点1 - 沿X轴正方向移动
    Waypoint point1("tool_axis_x_pos", 0.5, 0.0, 0.4, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(point1);
    
    // 点2 - 沿Y轴正方向移动
    Waypoint point2("tool_axis_y_pos", 0.5, 0.1, 0.4, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(point2);
    
    // 点3 - 沿Z轴负方向移动
    Waypoint point3("tool_axis_z_neg", 0.5, 0.1, 0.3, 0.0, 0.7071, 0.0, 0.7071, "base_link");
    waypoints.push_back(point3);
    
    // 回到起点
    waypoints.push_back(start);
    
    return waypoints;
}

} // namespace cr7_controller
