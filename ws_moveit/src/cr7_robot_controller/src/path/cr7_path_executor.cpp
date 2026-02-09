/**
 * @file cr7_path_executor.cpp
 * @brief 测试路径执行工具实现文件
 * 
 * 实现CR7PathExecutor类的所有方法，用于验证机器人的基本运动能力。
 */

#include <cmath>
#include <chrono>

#include "cr7_robot_controller/path/cr7_path_executor.hpp"
#include "cr7_robot_controller/base/cr7_base_controller.hpp"

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7PathExecutor 构造函数
// ============================================================================

/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param move_group MoveGroup接口指针
 * @param cartesian_planner 笛卡尔路径规划器指针
 * @param pilz_planner PILZ规划器指针
 * @param ompl_planner OMPL规划器指针
 */
CR7PathExecutor::CR7PathExecutor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    std::shared_ptr<CR7CartesianPlanner> cartesian_planner,
    std::shared_ptr<CR7PilzPlanner> pilz_planner,
    std::shared_ptr<CR7OMPLPlanner> ompl_planner
) : node_(node), move_group_(move_group), cartesian_planner_(cartesian_planner), pilz_planner_(pilz_planner), ompl_planner_(ompl_planner),
    logger_(node->get_logger())
{
    RCLCPP_INFO(logger_, "创建测试路径执行器");
}

// ============================================================================
// 测试路径执行方法
// ============================================================================

/**
 * @brief 执行测试路径
 * @return std::vector<CR7BaseController::Result> 每个路点的结果
 */
std::vector<CR7BaseController::Result> CR7PathExecutor::executeTestPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行基本测试路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    // 创建一个简单的测试路径
    std::vector<CR7BaseController::Result> results;
    
    try {
        // 获取当前位置
        auto current_pose = move_group_->getCurrentPose();
        RCLCPP_INFO(logger_, "当前位置: [%.3f, %.3f, %.3f]", 
                   current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        
        // 创建一个简单的测试路径点
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.z += 0.1; // 向上移动10cm
        
        // 优先使用OMPL规划器执行路径
        if (ompl_planner_) 
        {
            RCLCPP_INFO(logger_, "使用OMPL规划器执行测试路径...");
            auto result = ompl_planner_->moveToPose(target_pose, "test_waypoint");
            results.push_back(result);
        } 
        else 
        {
            //  fallback到直接使用MoveGroupInterface
            RCLCPP_INFO(logger_, "OMPL规划器未初始化，使用直接MoveGroupInterface执行测试路径...");
            // 设置目标位置
            move_group_->setPoseTarget(target_pose);
            
            // 规划运动
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            RCLCPP_INFO(logger_, "开始规划测试路径...");
            
            bool success = static_cast<bool>(move_group_->plan(plan));
            if (success) 
            {
                RCLCPP_INFO(logger_, "✓ 测试路径规划成功");
                RCLCPP_INFO(logger_, "轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
                
                // 执行运动
                RCLCPP_INFO(logger_, "开始执行测试路径...");
                auto result = move_group_->execute(plan);
                if (result == moveit::core::MoveItErrorCode::SUCCESS) 
                {
                    RCLCPP_INFO(logger_, "✓ 测试路径执行成功");
                    results.push_back(CR7BaseController::Result::SUCCESS);
                } 
                else 
                {
                    RCLCPP_ERROR(logger_, "测试路径执行失败");
                    results.push_back(CR7BaseController::Result::EXECUTION_FAILED);
                }
            } 
            else 
            {
                RCLCPP_ERROR(logger_, "测试路径规划失败");
                results.push_back(CR7BaseController::Result::PLANNING_FAILED);
            }
        }
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "执行测试路径异常: %s", e.what());
        results.push_back(CR7BaseController::Result::EXECUTION_FAILED);
    }
    
    return results;
}

/**
 * @brief 执行笛卡尔测试路径
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PathExecutor::executeCartesianTestPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行笛卡尔测试路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    try 
    {
        // 获取当前位置
        auto current_pose = move_group_->getCurrentPose();
        
        // 创建一个简单的笛卡尔路径
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose.pose);
        
        // 添加一个目标点
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x += 0.1; // 向前移动10cm
        target_pose.position.y += 0.1; // 向左移动10cm
        waypoints.push_back(target_pose);
        
        // 使用笛卡尔路径规划器执行路径
        if (cartesian_planner_) 
        {
            // 创建路径点
            std::vector<Waypoint> cartesian_waypoints;
            for (size_t i = 0; i < waypoints.size(); ++i) 
            {
                const auto& pose = waypoints[i];
                cartesian_waypoints.emplace_back(
                    "test_point_" + std::to_string(i),
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                );
            }
            
            // 使用优化的笛卡尔路径规划
            return cartesian_planner_->executeCartesianPath(cartesian_waypoints);
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "笛卡尔路径规划器未初始化");
            return CR7BaseController::Result::ROBOT_NOT_READY;
        }
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "执行笛卡尔测试路径异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

/**
 * @brief 执行PILZ测试路径
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PathExecutor::executePilzTestPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行PILZ测试路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    try 
    {
        // 获取当前位置
        auto current_pose = move_group_->getCurrentPose();
        
        // 创建一个简单的目标点
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x += 0.1; // 向前移动10cm
        target_pose.position.y += 0.1; // 向左移动10cm
        
        // 使用PILZ规划器执行直线运动
        if (pilz_planner_) 
        {
            return pilz_planner_->moveWithPilzLin(target_pose);
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "PILZ规划器未初始化");
            return CR7BaseController::Result::ROBOT_NOT_READY;
        }
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "执行PILZ测试路径异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

/**
 * @brief 执行工具坐标系测试路径
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PathExecutor::executeToolAxisTestPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行工具坐标系测试路径");
    RCLCPP_INFO(logger_, "=======================================");
    
    try 
    {
        // 获取当前位置
        // auto current_pose = move_group_->getCurrentPose();
        
        // 创建一个简单的工具坐标系测试路径
        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose current_pose = geometry_msgs::msg::Pose();
        current_pose.position.x = 0.0;
        current_pose.position.y = 0.0;
        current_pose.position.z = 0.0;
        current_pose.orientation.x = 0.0;
        current_pose.orientation.y = 0.0;
        current_pose.orientation.z = 0.0;
        current_pose.orientation.w = 1.0; // 单位四元数表示无旋转
        waypoints.push_back(current_pose);
        
        // 添加一个目标点
        // geometry_msgs::msg::Pose target_pose = current_pose.pose;
        geometry_msgs::msg::Pose target_pose = geometry_msgs::msg::Pose();
        target_pose.position.x = 0.0;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 1.0; // 单位四元数表示无旋转

        target_pose.position.z += 0.1; // 向上移动10cm
        waypoints.push_back(target_pose);
        
        // 使用笛卡尔路径规划器执行路径
        if (cartesian_planner_) 
        {
            // 创建路径点
            std::vector<Waypoint> cartesian_waypoints;
            for (size_t i = 0; i < waypoints.size(); ++i) 
            {
                const auto& pose = waypoints[i];
                cartesian_waypoints.emplace_back(
                    "tool_axis_test_point_" + std::to_string(i),
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, "welding_tcp"
                );// 设置为工具坐标系
            }
            
            // 使用优化的笛卡尔路径规划
            return cartesian_planner_->executeCartesianPath(cartesian_waypoints);
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "笛卡尔路径规划器未初始化");
            return CR7BaseController::Result::ROBOT_NOT_READY;
        }
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "执行工具坐标系测试路径异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

/**
 * @brief 执行焊接路径测试
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PathExecutor::executeWeldingTestPath()
{
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "开始执行焊接路径测试");
    RCLCPP_INFO(logger_, "=======================================");
    
    try {
        // 创建特定的焊接路径点
        // 起点
        auto start_wp = Waypoint("start_wp", 
                               0.77772, -0.3741, 0.028286, 
                               0.53928, 0.81723, 0.027122, 0.20142); 
        
        // 终点
        Waypoint end_wp = Waypoint("end_wp",
                                  0.77772, 0.37753, 0.028286,
                                  -0.37331, 0.8992, -0.084073, 0.21217);
        
        // 创建焊接路径
        std::vector<Waypoint> welding_waypoints;
        welding_waypoints.push_back(start_wp);
        welding_waypoints.push_back(end_wp);
        
        RCLCPP_INFO(logger_, "焊接路径点数量: %zu", welding_waypoints.size());
        RCLCPP_INFO(logger_, "起点位置: [%.3f, %.3f, %.3f]", 
                   start_wp.x, start_wp.y, start_wp.z);
        RCLCPP_INFO(logger_, "终点位置: [%.3f, %.3f, %.3f]", 
                   end_wp.x, end_wp.y, end_wp.z);
        
        // 使用OMPL规划器执行路径
        if (ompl_planner_) 
        {
            // 使用OMPL规划器执行路径
            RCLCPP_INFO(logger_, "使用OMPL规划器执行焊接路径测试");
            return ompl_planner_->executeWaypoints(welding_waypoints);
        } 
        else if (pilz_planner_) 
        {
            // 使用PILZ规划器执行路径
            RCLCPP_INFO(logger_, "使用PILZ规划器执行焊接路径测试");
            return pilz_planner_->moveWithPilzLin(end_wp.toPose());
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "没有可用的规划器");
            return CR7BaseController::Result::ROBOT_NOT_READY;
        }
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "执行焊接路径测试异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

} // namespace cr7_controller
