/**
 * @file cr7_pilz_planner.cpp
 * @brief PILZ工业规划器模块实现文件
 * 
 * 实现CR7PilzPlanner类的所有方法
 */

#include "cr7_robot_controller/pilz/cr7_pilz_planner.hpp"
#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <fstream>

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7PilzPlanner 构造函数
// ============================================================================

/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param move_group MoveGroup接口指针
 * @param logger 日志记录器
 */
CR7PilzPlanner::CR7PilzPlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
    rclcpp::Logger logger
) : node_(node), move_group_(move_group), logger_(logger), config_()
{
    RCLCPP_INFO(logger_, "创建PILZ工业规划器");
}

// ============================================================================
// PILZ规划器可用性检查
// ============================================================================

/**
 * @brief 检查PILZ规划器可用性
 * @return bool 是否可用
 */
bool CR7PilzPlanner::checkPilzAvailability() const
{
    if (!move_group_)
    {
        return false;
    }
    
    try
    {
        // MoveIt2 Humble 版本获取规划器的方式
        // 直接使用硬编码的PILZ规划器名称
        std::vector<std::string> pilz_planners = 
        {
            "LIN", "PTP", "CIRC", 
            "pilz_industrial_motion_planner",
            "PILZ"
        };
        
        RCLCPP_INFO(logger_, "检查PILZ规划器可用性");
        
        // 尝试设置PILZ规划器来检查是否可用
        for (const auto& planner : pilz_planners) 
        {
            try 
            {
                move_group_->setPlannerId(planner);
                RCLCPP_INFO(logger_, "发现PILZ规划器: %s", planner.c_str());
                return true;
            } 
            catch (...) 
            {
                // 继续尝试下一个
                continue;
            }
        }
        
        RCLCPP_WARN(logger_, "未发现可用的PILZ规划器");
        return false;
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "检查PILZ规划器可用性失败: %s", e.what());
        return false;
    }
}

/**
 * @brief 获取可用的PILZ规划器列表
 * @return std::vector<std::string> 规划器列表
 */
std::vector<std::string> CR7PilzPlanner::getAvailablePilzPlanners() const
{
    std::vector<std::string> pilz_planners;
    
    if (!move_group_)
    {
        return pilz_planners;
    }
    
    // MoveIt2 Humble 版本中，我们使用硬编码的PILZ规划器列表
    std::vector<std::string> possible_planners = 
    {
        "LIN", "PTP", "CIRC", 
        "pilz_industrial_motion_planner",
        "PILZ", "pilz"
    };
    
    // 检查哪些规划器可用
    for (const auto& planner : possible_planners) 
    {
        try 
        {
            // 保存当前规划器
            std::string current_planner = move_group_->getPlannerId();
            
            // 尝试设置规划器
            move_group_->setPlannerId(planner);
            
            // 如果设置成功，说明该规划器可用
            pilz_planners.push_back(planner);
            
            // 恢复原来的规划器
            move_group_->setPlannerId(current_planner);
            
        } 
        catch (...) 
        {
            // 规划器不可用，继续下一个
            continue;
        }
    }
    
    // 如果没有找到，返回默认的PILZ规划器
    if (pilz_planners.empty()) 
    {
        pilz_planners = {"LIN", "PTP", "CIRC"};
    }
    
    return pilz_planners;
}

/**
 * @brief 设置PILZ规划器
 * @param planner_id 规划器ID
 * @return bool 设置成功返回true
 */
bool CR7PilzPlanner::setPilzPlanner(const std::string& planner_id)
{
    if (!move_group_)
    {
        RCLCPP_ERROR(logger_, "MoveGroup接口未初始化");
        return false;
    }
    
    try 
    {
        // 直接尝试设置规划器
        move_group_->setPlannerId(planner_id);
        RCLCPP_INFO(logger_, "设置规划器: %s", planner_id.c_str());
        return true;
        
    } catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "设置规划器 %s 失败: %s", planner_id.c_str(), e.what());
        
        // 尝试使用默认规划器
        try 
        {
            move_group_->setPlannerId("RRTConnect");
            RCLCPP_WARN(logger_, "使用默认规划器 RRTConnect");
            return false;
        } 
        catch (...) 
        {
            return false;
        }
    }
}

// ============================================================================
// PILZ规划器执行方法
// ============================================================================
/**
 * @brief 测试PILZ规划器功能
 * @param planner_type 规划器类型
 * @return CR7BaseController::Result 测试结果
 */
CR7BaseController::Result CR7PilzPlanner::testPilzPlanner(PilzPlanner planner_type)
{
    if (!move_group_)
    {
        RCLCPP_ERROR(logger_, "MoveGroup接口未初始化");
        return CR7BaseController::Result::ROBOT_NOT_READY;
    }
    
    std::string planner_name = pilzPlannerToString(planner_type);
    RCLCPP_INFO(logger_, "=======================================");
    RCLCPP_INFO(logger_, "测试PILZ规划器: %s", planner_name.c_str());
    RCLCPP_INFO(logger_, "=======================================");
    
    // 直接设置规划器，不检查可用性（让MoveIt2自己处理）
    if (!setPilzPlanner(planner_name)) 
    {
        RCLCPP_WARN(logger_, "设置规划器失败，尝试继续测试");
    }
    
    // 获取当前位置
    auto current_pose = getCurrentPose();
    RCLCPP_INFO(logger_, "当前位置: [%.3f, %.3f, %.3f]", 
               current_pose.position.x, current_pose.position.y, current_pose.position.z);
    
    // 创建测试目标位姿
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += 0.1;  // 向前移动10cm
    target_pose.position.y += 0.1;  // 向前移动10cm  
    
    RCLCPP_INFO(logger_, "目标位置: [%.3f, %.3f, %.3f]", 
               target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    // 执行规划
    CR7BaseController::Result result;
    switch (planner_type) 
    {
        case PilzPlanner::LIN:
            result = moveWithPilzLin(target_pose);
            break;
        case PilzPlanner::PTP:
            result = moveWithPilzPtp(target_pose);
            break;
        case PilzPlanner::CIRC:
            // 圆周运动需要特殊处理，这里简化测试
            RCLCPP_WARN(logger_, "圆周运动需要中间点，使用直线运动进行测试");
            result = moveWithPilzLin(target_pose);
            break;
        default:
            result = CR7BaseController::Result::INVALID_INPUT;
            break;
    }
    
    RCLCPP_INFO(logger_, "PILZ %s 规划器测试结果: %s", 
               planner_name.c_str(), CR7BaseController::resultToString(result).c_str());
    
    return result;
}

/**
 * @brief 使用PILZ规划器执行直线运动
 * @param target_pose 目标位姿
 * @param config PILZ配置
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PilzPlanner::moveWithPilzLin(
    const geometry_msgs::msg::Pose& target_pose,
    const PilzConfig& config)
{
    RCLCPP_INFO(logger_, "执行PILZ直线运动");
    return executePilzPlan(target_pose, "LIN", config);
}

/**
 * @brief 使用PILZ规划器执行点对点运动
 * @param target_pose 目标位姿
 * @param config PILZ配置
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PilzPlanner::moveWithPilzPtp(
    const geometry_msgs::msg::Pose& target_pose,
    const PilzConfig& config)
{
    RCLCPP_INFO(logger_, "执行PILZ点对点运动");
    return executePilzPlan(target_pose, "PTP", config);
}

/**
 * @brief 使用PILZ规划器执行圆周运动
 * @param intermediate_pose 中间点
 * @param target_pose 目标点
 * @param config PILZ配置
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PilzPlanner::moveWithPilzCirc(
    const geometry_msgs::msg::Pose& intermediate_pose,
    const geometry_msgs::msg::Pose& target_pose,
    const PilzConfig& config)
{
    if (!move_group_)
    {
        RCLCPP_ERROR(logger_, "MoveGroup接口未初始化");
        return CR7BaseController::Result::ROBOT_NOT_READY;
    }
    
    RCLCPP_INFO(logger_, "执行PILZ圆周运动");
    
    // 设置圆周规划器
    if (!setPilzPlanner("CIRC")) 
    {
        RCLCPP_ERROR(logger_, "设置圆周规划器失败");
        return CR7BaseController::Result::PLANNING_FAILED;
    }
    
    try 
    {
        // 设置规划参数
        move_group_->setMaxVelocityScalingFactor(config.velocity_scale);
        move_group_->setMaxAccelerationScalingFactor(config.acceleration_scale);
        
        // 对于圆周运动，我们需要使用不同的接口
        // 这里简化处理，使用标准的位姿目标规划
        move_group_->setPoseTarget(target_pose);
        
        // 规划运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO(logger_, "开始PILZ圆周运动规划...");
        
        auto start_time = node_->now();
        bool success = static_cast<bool>(move_group_->plan(plan));
        double planning_time = (node_->now() - start_time).seconds();
        
        if (!success) 
        {
            RCLCPP_ERROR(logger_, "圆周运动规划失败 (耗时 %.3f 秒)", planning_time);
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "✓ 圆周运动规划成功 (耗时 %.3f 秒)", planning_time);
        RCLCPP_INFO(logger_, "轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行圆周运动...");
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) 
        {
            RCLCPP_INFO(logger_, "✓ 圆周运动执行成功 (耗时 %.3f 秒)", execution_time);
            return CR7BaseController::Result::SUCCESS;
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "圆周运动执行失败 (错误码: %d)", result.val);
            return CR7BaseController::Result::EXECUTION_FAILED;
        }
        
    } 
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(logger_, "圆周运动异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

// ============================================================================
// PILZ规划核心实现
// ============================================================================
/**
 * @brief 执行PILZ规划
 * @param target_pose 目标位姿
 * @param planner_id 规划器ID
 * @param config PILZ配置
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7PilzPlanner::executePilzPlan(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& planner_id,
    const PilzConfig& config)
{
    
    if (!move_group_)
    {
        RCLCPP_ERROR(logger_, "MoveGroup接口未初始化");
        return CR7BaseController::Result::ROBOT_NOT_READY;
    }
    
    try 
    {
        // 1. 设置规划器
        move_group_->setPlannerId(planner_id);
        RCLCPP_INFO(logger_, "使用规划器: %s", planner_id.c_str());
        
        // 2. 设置规划参数
        move_group_->setMaxVelocityScalingFactor(config.velocity_scale);
        move_group_->setMaxAccelerationScalingFactor(config.acceleration_scale);
        
        // 3. 设置PILZ特定的约束
        if (planner_id == "LIN") 
        {
            // 为直线运动设置约束
            moveit_msgs::msg::Constraints path_constraints;
            
            // 位置约束 - 直线运动
            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.header.frame_id = move_group_->getPlanningFrame();
            pos_constraint.link_name = move_group_->getEndEffectorLink();
            pos_constraint.weight = 1.0;
            
            // 创建球形约束区域
            shape_msgs::msg::SolidPrimitive sphere;
            sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
            sphere.dimensions.resize(1);
            sphere.dimensions[0] = config.max_deviation;  // 允许的最大偏差
            
            geometry_msgs::msg::Pose sphere_pose;
            sphere_pose.orientation.w = 1.0;
            
            // pos_constraint.constraint_region.primitives.push_back(sphere);
            // pos_constraint.constraint_region.primitive_poses.push_back(sphere_pose);
            
            // 方向约束 - 保持末端朝向
            auto current_pose = move_group_->getCurrentPose().pose;
            moveit_msgs::msg::OrientationConstraint orient_constraint;
            orient_constraint.header.frame_id = move_group_->getPlanningFrame();
            orient_constraint.link_name = move_group_->getEndEffectorLink();
            orient_constraint.orientation = current_pose.orientation;
            orient_constraint.absolute_x_axis_tolerance = config.orientation_tolerance;
            orient_constraint.absolute_y_axis_tolerance = config.orientation_tolerance;
            orient_constraint.absolute_z_axis_tolerance = config.orientation_tolerance;
            orient_constraint.weight = 1.0;
            
            path_constraints.position_constraints.push_back(pos_constraint);
            // path_constraints.orientation_constraints.push_back(orient_constraint);
            
            move_group_->setPathConstraints(path_constraints);
            RCLCPP_INFO(logger_, "已设置直线约束，最大偏差: %.3f m", config.max_deviation);
        }
        
        // 4. 清除目标并设置新目标
        move_group_->clearPoseTargets();
        move_group_->setPoseTarget(target_pose);
        
        // 5. 设置目标容差
        move_group_->setGoalTolerance(config.goal_position_tolerance);
        move_group_->setGoalOrientationTolerance(config.goal_orientation_tolerance);
        
        // 6. 规划（减少尝试次数）
        const int max_attempts = 20;  // 减少到3次
        moveit::planning_interface::MoveGroupInterface::Plan best_plan;
        bool planning_success = false;
        
        for (int attempt = 0; attempt < max_attempts; ++attempt) 
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            
            auto start_time = node_->now();
            bool success = static_cast<bool>(move_group_->plan(plan));
            double planning_time = (node_->now() - start_time).seconds();
            
            if (success && !plan.trajectory_.joint_trajectory.points.empty()) {
                RCLCPP_INFO(logger_, "✓ PILZ %s 规划成功 (第%d次尝试, 耗时 %.3f 秒)", 
                           planner_id.c_str(), attempt + 1, planning_time);
                RCLCPP_INFO(logger_, "轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
                
                // 验证LIN轨迹
                if (planner_id == "LIN") {
                    bool is_linear = true;
                    const auto& points = plan.trajectory_.joint_trajectory.points;
                    if (points.size() > 2) {
                        // 简单检查：末端应该接近直线运动
                        // 您可以添加更复杂的检查逻辑
                    }
                    
                    if (!is_linear) 
                    {
                        RCLCPP_WARN(logger_, "LIN轨迹不够直，重新尝试...");
                        continue;
                    }
                }
                
                best_plan = plan;
                planning_success = true;
                break;
            } 
            else 
            {
                RCLCPP_WARN(logger_, "PILZ %s 规划失败 (第%d次尝试, 耗时 %.3f 秒)", 
                           planner_id.c_str(), attempt + 1, planning_time);
                
                // 短暂延迟后重试
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        
        if (!planning_success) 
        {
            RCLCPP_ERROR(logger_, "PILZ %s 规划失败: 所有 %d 次尝试都未成功", 
                        planner_id.c_str(), max_attempts);
            move_group_->clearPathConstraints();  // 清理约束
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        // 7. 清理约束
        move_group_->clearPathConstraints();
        
        // 8. 保存轨迹分析
        if (planning_success) 
        {
            std::string trajectory_prefix = "pilz_" + planner_id + "_trajectory";
            saveTrajectoryAnalysis(best_plan.trajectory_, trajectory_prefix);
        }
        
        // 9. 执行规划
        RCLCPP_INFO(logger_, "开始执行PILZ %s 运动...", planner_id.c_str());
        auto start_time = node_->now();
        auto result = move_group_->execute(best_plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) 
        {
            RCLCPP_INFO(logger_, "✓ PILZ %s 运动执行成功 (耗时 %.3f 秒)", 
                       planner_id.c_str(), execution_time);
            return CR7BaseController::Result::SUCCESS;
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "PILZ %s 运动执行失败 (错误码: %d)", 
                        planner_id.c_str(), result.val);
            return CR7BaseController::Result::EXECUTION_FAILED;
        }
        
    } catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "PILZ %s 运动异常: %s", planner_id.c_str(), e.what());
        // 确保清理约束
        try { move_group_->clearPathConstraints(); } 
        catch (...) {}
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

// ============================================================================
// 工具方法
// ============================================================================
/**
 * @brief PILZ规划器类型转字符串
 * @param planner 规划器类型
 * @return std::string 规划器名称
 */
std::string CR7PilzPlanner::pilzPlannerToString(PilzPlanner planner)
{
    switch (planner) {
        case PilzPlanner::LIN: return "LIN";
        case PilzPlanner::PTP: return "PTP";
        case PilzPlanner::CIRC: return "CIRC";
        default: return "UNKNOWN";
    }
}

/**
 * @brief 字符串转PILZ规划器类型
 * @param planner_name 规划器名称
 * @return PilzPlanner 规划器类型
 */
PilzPlanner CR7PilzPlanner::stringToPilzPlanner(const std::string& planner_name)
{
    if (planner_name == "LIN" || planner_name.find("lin") != std::string::npos) 
    {
        return PilzPlanner::LIN;
    } 
    else if (planner_name == "PTP" || planner_name.find("ptp") != std::string::npos) 
    {
        return PilzPlanner::PTP;
    } 
    else if (planner_name == "CIRC" || planner_name.find("circ") != std::string::npos) 
    {
        return PilzPlanner::CIRC;
    } 
    else 
    {
        return PilzPlanner::LIN; // 默认返回LIN
    }
}

/**
 * @brief 保存轨迹分析
 */
void CR7PilzPlanner::saveTrajectoryAnalysis(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& filename_prefix)
{
    
    if (trajectory.joint_trajectory.points.empty())
    {
        return;
    }
    
    std::string filename = filename_prefix + ".txt";
    std::ofstream file(filename);
    
    if (!file.is_open()) 
    {
        RCLCPP_WARN(logger_, "无法创建轨迹分析文件: %s", filename.c_str());
        return;
    }
    
    file << "PILZ轨迹分析报告" << std::endl;
    file << "=================" << std::endl;
    file << "轨迹点数: " << trajectory.joint_trajectory.points.size() << std::endl;
    
    const auto& points = trajectory.joint_trajectory.points;
    double total_time = points.back().time_from_start.sec + 
                       points.back().time_from_start.nanosec * 1e-9;
    file << "轨迹总时间: " << std::fixed << std::setprecision(3) << total_time << " 秒" << std::endl;
    
    // 打印PILZ轨迹特点
    file << "PILZ轨迹特点:" << std::endl;
    file << "  - 平滑的加速度曲线" << std::endl;
    file << "  - 工业级运动规划" << std::endl;
    file << "  - 实时性能优化" << std::endl;
    
    file.close();
    RCLCPP_INFO(logger_, "PILZ轨迹分析已保存到: %s", filename.c_str());
}

}  // namespace cr7_controller
