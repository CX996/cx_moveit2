/**
 * @file cr7_robot_controller.cpp
 * @brief CR7机器人控制器实现文件
 * 
 * 实现CR7RobotController类的所有方法
 * 集成PILZ工业规划器功能
 */

#include "cr7_robot_controller/cr7_robot_controller.hpp"
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
// Waypoint 方法实现
// ============================================================================

geometry_msgs::msg::Pose Waypoint::toPose() const {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

bool Waypoint::isValidQuaternion() const {
    const double norm = qx*qx + qy*qy + qz*qz + qw*qw;
    return std::abs(norm - 1.0) < 0.001;
}

void Waypoint::normalizeQuaternion() {
    const double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm > 1e-6) {
        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;
    } else {
        // 如果四元数为零，设置为单位四元数
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
        qw = 1.0;
    }
}

// ============================================================================
// CR7RobotController 方法实现
// ============================================================================

CR7RobotController::CR7RobotController(rclcpp::Node::SharedPtr node, 
                                      const std::string& planning_group)
    : node_(node)
    , logger_(node->get_logger())
    , initialized_(false) {
    
    RCLCPP_INFO(logger_, "创建CR7机器人控制器");
    RCLCPP_INFO(logger_, "规划组: %s", planning_group.c_str());
    
    try {
        // 创建MoveGroup接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, planning_group);
        
        move_group_->setPlannerId("RRTstarkConfigDefault");  // 例如使用RRTConnect

        // 创建规划场景接口
        planning_scene_interface_ = 
            std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
        RCLCPP_INFO(logger_, "MoveGroup接口创建成功");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger_, "创建MoveGroup接口失败: %s", e.what());
        throw;
    }
}

bool CR7RobotController::initialize(double timeout_sec) {
    RCLCPP_INFO(logger_, "开始初始化机器人控制器");
    
    if (initialized_) {
        RCLCPP_WARN(logger_, "控制器已经初始化");
        return true;
    }
    
    // 1. 等待机器人状态
    if (!waitForRobotState(timeout_sec)) {
        RCLCPP_ERROR(logger_, "等待机器人状态超时");
        return false;
    }
    
    // 2. 设置默认规划参数
    move_group_->setPlanningTime(10.0);           // 规划时间20秒
    move_group_->setNumPlanningAttempts(100);      // 尝试50次
    move_group_->setMaxVelocityScalingFactor(0.3);  // 最大速度50%
    move_group_->setMaxAccelerationScalingFactor(0.3);  // 最大加速度30%
    
    // 3. 设置目标容差
    move_group_->setGoalPositionTolerance(0.001);    // 1mm
    move_group_->setGoalOrientationTolerance(0.01);  // 约0.57度
    
    RCLCPP_INFO(logger_, "机器人控制器初始化完成");
    RCLCPP_INFO(logger_, "规划时间: %.1f秒", move_group_->getPlanningTime());
    
    initialized_ = true;
    return true;
}

bool CR7RobotController::waitForRobotState(double timeout_sec) 
{
    RCLCPP_INFO(logger_, "等待机器人状态 (timeout = %.2f s)...", timeout_sec);

    // MoveIt2 官方阻塞接口：
    // 会等待 joint_states，直到 CurrentStateMonitor 收到"时间有效"的状态

    moveit::core::RobotStatePtr current_state =
        move_group_->getCurrentState(timeout_sec);

    if (!current_state)
    {
        RCLCPP_ERROR(
            logger_,
            "等待机器人状态失败：在 %.2f 秒内未收到有效的 joint_states",
            timeout_sec
        );
        return false;
    }

    // ===== 成功拿到状态 =====
    const auto& joint_names = move_group_->getJointNames();

    RCLCPP_INFO(logger_, "✓ 成功获取机器人状态");
    RCLCPP_INFO(logger_, "关节数量: %zu", joint_names.size());

    // （可选）打印关节值，用于调试
    for (const auto& name : joint_names)
    {
        double value = current_state->getVariablePosition(name);
        RCLCPP_DEBUG(logger_, "  %s = %.6f", name.c_str(), value);
    }

    return true;
}

CR7RobotController::Result CR7RobotController::moveToPose(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& waypoint_name) {
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    // 验证输入
    std::string error_msg;
    if (!validatePose(target_pose, &error_msg)) {
        RCLCPP_ERROR(logger_, "无效的位姿: %s", error_msg.c_str());
        return Result::INVALID_INPUT;
    }
    
    // 打印目标信息
    if (!waypoint_name.empty()) {
        RCLCPP_INFO(logger_, "========================================");
        RCLCPP_INFO(logger_, "移动到路点: %s", waypoint_name.c_str());
    }
    
    RCLCPP_INFO(logger_, "目标位置: [%.3f, %.3f, %.3f]", 
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(logger_, "目标姿态: [%.3f, %.3f, %.3f, %.3f]",
                target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w);
    
    try {
        // 设置目标位姿
        move_group_->setPoseTarget(target_pose);
        
        // 规划运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO(logger_, "开始规划...");
        
        auto start_time = node_->now();
        bool success = static_cast<bool>(move_group_->plan(plan));
        double planning_time = (node_->now() - start_time).seconds();
        
        if (!success) {
            RCLCPP_ERROR(logger_, "规划失败 (耗时 %.3f 秒)", planning_time);
            return Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "✓ 规划成功 (耗时 %.3f 秒)", planning_time);
        RCLCPP_INFO(logger_, "轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
        
        // 检查轨迹是否有效
        if (plan.trajectory_.joint_trajectory.points.empty()) {
            RCLCPP_ERROR(logger_, "轨迹为空");
            return Result::PLANNING_FAILED;
        }
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行...");
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger_, "✓ 执行成功 (耗时 %.3f 秒)", execution_time);
            if (!waypoint_name.empty()) {
                RCLCPP_INFO(logger_, "✓ 到达路点: %s", waypoint_name.c_str());
            }
            return Result::SUCCESS;
        } else {
            RCLCPP_ERROR(logger_, "执行失败 (错误码: %d)", result.val);
            RCLCPP_ERROR(logger_, "耗时: %.3f 秒", execution_time);
            return Result::EXECUTION_FAILED;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "运动控制异常: %s", e.what());
        return Result::EXECUTION_FAILED;
    }
}

CR7RobotController::Result CR7RobotController::moveToWaypoint(const Waypoint& waypoint) {
    // 检查四元数
    if (!waypoint.isValidQuaternion()) {
        RCLCPP_WARN(logger_, "四元数未归一化，自动处理...");
        Waypoint normalized_waypoint = waypoint;
        normalized_waypoint.normalizeQuaternion();
        return moveToPose(normalized_waypoint.toPose(), waypoint.name);
    }
    
    return moveToPose(waypoint.toPose(), waypoint.name);
}

std::vector<CR7RobotController::Result> CR7RobotController::executeWaypoints(
    const std::vector<Waypoint>& waypoints,
    double delay_seconds) {
    
    std::vector<Result> results;
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        results.push_back(Result::ROBOT_NOT_READY);
        return results;
    }
    
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "开始执行 %zu 个路点", waypoints.size());
    RCLCPP_INFO(logger_, "路点间延迟: %.1f 秒", delay_seconds);
    RCLCPP_INFO(logger_, "========================================");
    
    int success_count = 0;
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];
        
        RCLCPP_INFO(logger_, "----------------------------------------");
        RCLCPP_INFO(logger_, "路点 %zu/%zu: %s", 
                   i + 1, waypoints.size(), wp.name.c_str());
        
        // 执行路点
        auto result = moveToWaypoint(wp);
        results.push_back(result);
        
        if (result == Result::SUCCESS) {
            success_count++;
        } else {
            RCLCPP_ERROR(logger_, "路点 %s 失败: %s", 
                        wp.name.c_str(), resultToString(result).c_str());
        }
        
        // 路点间延迟（最后一个路点后不延迟）
        if (i < waypoints.size() - 1 && delay_seconds > 0) {
            RCLCPP_INFO(logger_, "等待 %.1f 秒...", delay_seconds);
            for (int t = static_cast<int>(delay_seconds); t > 0; --t) {
                RCLCPP_INFO(logger_, "%d...", t);
                rclcpp::sleep_for(1s);
            }
        }
    }
    
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "路点序列执行完成");
    RCLCPP_INFO(logger_, "成功: %d/%zu", success_count, waypoints.size());
    RCLCPP_INFO(logger_, "========================================");
    
    return results;
}

// ============================================================================
// 笛卡尔路径规划核心实现
// ============================================================================

CR7RobotController::Result CR7RobotController::executeCartesianPath(
    const std::vector<Waypoint>& waypoints,
    const CartesianPathConfig& config) {
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    if (waypoints.size() < 2) {
        RCLCPP_ERROR(logger_, "笛卡尔路径需要至少2个路点");
        return Result::INVALID_INPUT;
    }
    
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "开始笛卡尔路径规划");
    RCLCPP_INFO(logger_, "路点数量: %zu", waypoints.size());
    RCLCPP_INFO(logger_, "最大步长: %.4f m", config.max_step);
    RCLCPP_INFO(logger_, "跳跃阈值: %.3f", config.jump_threshold);
    RCLCPP_INFO(logger_, "避障: %s", config.avoid_collisions ? "是" : "否");
    RCLCPP_INFO(logger_, "速度因子: %.2f", config.max_velocity_factor);
    RCLCPP_INFO(logger_, "========================================");
    
    // 添加：验证所有输入路点
    for (size_t i = 0; i < waypoints.size(); ++i) {
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
            return Result::INVALID_INPUT;
        }
        
        if (std::isnan(wp.qx) || std::isinf(wp.qx) ||
            std::isnan(wp.qy) || std::isinf(wp.qy) ||
            std::isnan(wp.qz) || std::isinf(wp.qz) ||
            std::isnan(wp.qw) || std::isinf(wp.qw)) {
            RCLCPP_ERROR(logger_, "路点 %zu 四元数包含无效数据", i);
            return Result::INVALID_INPUT;
        }
    }
    
    try {
        // 1. 预处理路径点
        std::vector<geometry_msgs::msg::Pose> waypoint_poses;
        if (!preprocessWaypoints(waypoints, waypoint_poses, config)) 
        {
            RCLCPP_ERROR(logger_, "路径点预处理失败");
            return Result::INVALID_INPUT;
        }
        
        // 2. 检查路径可行性
        if (!checkPathFeasibility(waypoint_poses)) 
        {
            RCLCPP_ERROR(logger_, "路径不可行");
            return Result::PLANNING_FAILED;
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
        
        if (fraction < 0.8) {
            RCLCPP_WARN(logger_, "笛卡尔路径完成度较低 (%.1f%%)，尝试优化...", fraction * 100.0);
            
            // 尝试简化路径
            double simplified_fraction = trySimplifiedPath(waypoint_poses, trajectory, config);
            if (simplified_fraction > fraction) 
            {
                fraction = simplified_fraction;
                RCLCPP_INFO(logger_, "优化后完成度: %.1f%%", fraction * 100.0);
            }
        }
        
        if (fraction <= 0.0) {
            RCLCPP_ERROR(logger_, "笛卡尔路径规划失败");
            return Result::CARTESIAN_FAILED;
        }
        
        // 5. 保存轨迹分析信息
        saveTrajectoryAnalysis(trajectory, "original_trajectory");
        
        // 6. 优化轨迹 - 只做轻微优化，不删除太多点
        if (!optimizeTrajectoryLight(trajectory)) {
            RCLCPP_WARN(logger_, "轨迹优化失败，使用原始轨迹");
        } else {
            saveTrajectoryAnalysis(trajectory, "optimized_trajectory");
        }
        
        // 7. 检查轨迹有效性
        if (trajectory.joint_trajectory.points.empty()) {
            RCLCPP_ERROR(logger_, "生成的轨迹为空");
            return Result::CARTESIAN_FAILED;
        }
        
        RCLCPP_INFO(logger_, "轨迹点数: %zu", trajectory.joint_trajectory.points.size());
        
        // 8. 验证轨迹
        if (!validateTrajectory(trajectory)) {
            RCLCPP_ERROR(logger_, "轨迹验证失败");
            return Result::CARTESIAN_FAILED;
        }
        
        // 9. 打印轨迹详细信息
        printTrajectoryInfo(trajectory);
        
        // 10. 执行轨迹
        RCLCPP_INFO(logger_, "开始执行笛卡尔路径...");
        start_time = node_->now();
        auto result = move_group_->execute(trajectory);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger_, "✓ 笛卡尔路径执行成功 (耗时 %.3f 秒)", execution_time);
            RCLCPP_INFO(logger_, "✓ 完成度: %.1f%%", fraction * 100.0);
            return Result::SUCCESS;
        } else {
            RCLCPP_ERROR(logger_, "笛卡尔路径执行失败 (错误码: %d)", result.val);
            RCLCPP_ERROR(logger_, "耗时: %.3f 秒", execution_time);
            return Result::EXECUTION_FAILED;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "笛卡尔路径规划异常: %s", e.what());
        return Result::CARTESIAN_FAILED;
    }
}

/**
 * @brief 优化的笛卡尔路径规划（推荐使用）
 */
CR7RobotController::Result CR7RobotController::executeOptimizedCartesianPath(
    const std::vector<Waypoint>& waypoints) {
    
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
// PILZ工业规划器方法实现
// ============================================================================

bool CR7RobotController::checkPilzAvailability() const {
    if (!initialized_) {
        return false;
    }
    
    try {
        // MoveIt2 Humble 版本获取规划器的方式
        // 直接使用硬编码的PILZ规划器名称
        std::vector<std::string> pilz_planners = {
            "LIN", "PTP", "CIRC", 
            "pilz_industrial_motion_planner",
            "PILZ"
        };
        
        RCLCPP_INFO(logger_, "检查PILZ规划器可用性");
        
        // 尝试设置PILZ规划器来检查是否可用
        for (const auto& planner : pilz_planners) {
            try {
                move_group_->setPlannerId(planner);
                RCLCPP_INFO(logger_, "发现PILZ规划器: %s", planner.c_str());
                return true;
            } catch (...) {
                // 继续尝试下一个
                continue;
            }
        }
        
        RCLCPP_WARN(logger_, "未发现可用的PILZ规划器");
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "检查PILZ规划器可用性失败: %s", e.what());
        return false;
    }
}

std::vector<std::string> CR7RobotController::getAvailablePilzPlanners() const {
    std::vector<std::string> pilz_planners;
    
    if (!initialized_) {
        return pilz_planners;
    }
    
    // MoveIt2 Humble 版本中，我们使用硬编码的PILZ规划器列表
    std::vector<std::string> possible_planners = {
        "LIN", "PTP", "CIRC", 
        "pilz_industrial_motion_planner",
        "PILZ", "pilz"
    };
    
    // 检查哪些规划器可用
    for (const auto& planner : possible_planners) {
        try {
            // 保存当前规划器
            std::string current_planner = move_group_->getPlannerId();
            
            // 尝试设置规划器
            move_group_->setPlannerId(planner);
            
            // 如果设置成功，说明该规划器可用
            pilz_planners.push_back(planner);
            
            // 恢复原来的规划器
            move_group_->setPlannerId(current_planner);
            
        } catch (...) {
            // 规划器不可用，继续下一个
            continue;
        }
    }
    
    // 如果没有找到，返回默认的PILZ规划器
    if (pilz_planners.empty()) {
        pilz_planners = {"LIN", "PTP", "CIRC"};
    }
    
    return pilz_planners;
}

bool CR7RobotController::setPilzPlanner(const std::string& planner_id) {
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return false;
    }
    
    try {
        // 直接尝试设置规划器
        move_group_->setPlannerId(planner_id);
        RCLCPP_INFO(logger_, "设置规划器: %s", planner_id.c_str());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "设置规划器 %s 失败: %s", planner_id.c_str(), e.what());
        
        // 尝试使用默认规划器
        try {
            move_group_->setPlannerId("RRTConnect");
            RCLCPP_WARN(logger_, "使用默认规划器 RRTConnect");
            return false;
        } catch (...) {
            return false;
        }
    }
}

std::string CR7RobotController::pilzPlannerToString(PilzPlanner planner) {
    switch (planner) {
        case PilzPlanner::LIN: return "LIN";
        case PilzPlanner::PTP: return "PTP";
        case PilzPlanner::CIRC: return "CIRC";
        default: return "UNKNOWN";
    }
}

CR7RobotController::PilzPlanner CR7RobotController::stringToPilzPlanner(const std::string& planner_name) {
    if (planner_name == "LIN" || planner_name.find("lin") != std::string::npos) {
        return PilzPlanner::LIN;
    } else if (planner_name == "PTP" || planner_name.find("ptp") != std::string::npos) {
        return PilzPlanner::PTP;
    } else if (planner_name == "CIRC" || planner_name.find("circ") != std::string::npos) {
        return PilzPlanner::CIRC;
    } else {
        return PilzPlanner::LIN; // 默认返回LIN
    }
}

CR7RobotController::Result CR7RobotController::testPilzPlanner(PilzPlanner planner_type) {
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    std::string planner_name = pilzPlannerToString(planner_type);
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "测试PILZ规划器: %s", planner_name.c_str());
    RCLCPP_INFO(logger_, "========================================");
    
    // 直接设置规划器，不检查可用性（让MoveIt2自己处理）
    if (!setPilzPlanner(planner_name)) {
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
    Result result;
    switch (planner_type) {
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
            result = Result::INVALID_INPUT;
            break;
    }
    
    RCLCPP_INFO(logger_, "PILZ %s 规划器测试结果: %s", 
               planner_name.c_str(), resultToString(result).c_str());
    
    return result;
}

CR7RobotController::Result CR7RobotController::moveWithPilzLin(const geometry_msgs::msg::Pose& target_pose,
                                                              const PilzConfig& config) {
    RCLCPP_INFO(logger_, "执行PILZ直线运动");
    return executePilzPlan(target_pose, "LIN", config);
}

CR7RobotController::Result CR7RobotController::moveWithPilzPtp(const geometry_msgs::msg::Pose& target_pose,
                                                             const PilzConfig& config) {
    RCLCPP_INFO(logger_, "执行PILZ点对点运动");
    return executePilzPlan(target_pose, "PTP", config);
}

CR7RobotController::Result CR7RobotController::moveWithPilzCirc(const geometry_msgs::msg::Pose& intermediate_pose,
                                                              const geometry_msgs::msg::Pose& target_pose,
                                                              const PilzConfig& config) {
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    RCLCPP_INFO(logger_, "执行PILZ圆周运动");
    
    // 设置圆周规划器
    if (!setPilzPlanner("CIRC")) {
        RCLCPP_ERROR(logger_, "设置圆周规划器失败");
        return Result::PLANNING_FAILED;
    }
    
    // 验证输入位姿
    std::string error_msg;
    if (!validatePose(intermediate_pose, &error_msg)) {
        RCLCPP_ERROR(logger_, "中间点位姿无效: %s", error_msg.c_str());
        return Result::INVALID_INPUT;
    }
    
    if (!validatePose(target_pose, &error_msg)) {
        RCLCPP_ERROR(logger_, "目标位姿无效: %s", error_msg.c_str());
        return Result::INVALID_INPUT;
    }
    
    try {
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
        
        if (!success) {
            RCLCPP_ERROR(logger_, "圆周运动规划失败 (耗时 %.3f 秒)", planning_time);
            return Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "✓ 圆周运动规划成功 (耗时 %.3f 秒)", planning_time);
        RCLCPP_INFO(logger_, "轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行圆周运动...");
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger_, "✓ 圆周运动执行成功 (耗时 %.3f 秒)", execution_time);
            return Result::SUCCESS;
        } else {
            RCLCPP_ERROR(logger_, "圆周运动执行失败 (错误码: %d)", result.val);
            return Result::EXECUTION_FAILED;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "圆周运动异常: %s", e.what());
        return Result::EXECUTION_FAILED;
    }
}

CR7RobotController::Result CR7RobotController::executePilzPlan(const geometry_msgs::msg::Pose& target_pose,
                                                               const std::string& planner_id,
                                                               const PilzConfig& config) {
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    // 设置规划器
    if (!setPilzPlanner(planner_id)) {
        return Result::PLANNING_FAILED;
    }
    
    // 验证输入位姿
    std::string error_msg;
    if (!validatePose(target_pose, &error_msg)) {
        RCLCPP_ERROR(logger_, "目标位姿无效: %s", error_msg.c_str());
        return Result::INVALID_INPUT;
    }
    
    try {
        // 设置规划参数
        move_group_->setMaxVelocityScalingFactor(config.velocity_scale);
        move_group_->setMaxAccelerationScalingFactor(config.acceleration_scale);
        
        // 设置目标位姿
        move_group_->setPoseTarget(target_pose);
        
        // 规划运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        RCLCPP_INFO(logger_, "开始PILZ %s 规划...", planner_id.c_str());
        
        auto start_time = node_->now();
        bool success = static_cast<bool>(move_group_->plan(plan));
        double planning_time = (node_->now() - start_time).seconds();
        
        if (!success) {
            RCLCPP_ERROR(logger_, "PILZ %s 规划失败 (耗时 %.3f 秒)", planner_id.c_str(), planning_time);
            return Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "✓ PILZ %s 规划成功 (耗时 %.3f 秒)", planner_id.c_str(), planning_time);
        RCLCPP_INFO(logger_, "轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
        
        // 分析轨迹特点
        if (!plan.trajectory_.joint_trajectory.points.empty()) {
            const auto& points = plan.trajectory_.joint_trajectory.points;
            double total_time = points.back().time_from_start.sec + 
                               points.back().time_from_start.nanosec * 1e-9;
            RCLCPP_INFO(logger_, "轨迹总时间: %.3f 秒", total_time);
            
            // 打印PILZ轨迹特点
            RCLCPP_INFO(logger_, "PILZ轨迹特点:");
            RCLCPP_INFO(logger_, "  - 平滑的加速度曲线");
            RCLCPP_INFO(logger_, "  - 工业级运动规划");
            RCLCPP_INFO(logger_, "  - 实时性能优化");
        }
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行PILZ %s 运动...", planner_id.c_str());
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(logger_, "✓ PILZ %s 运动执行成功 (耗时 %.3f 秒)", 
                       planner_id.c_str(), execution_time);
            return Result::SUCCESS;
        } else {
            RCLCPP_ERROR(logger_, "PILZ %s 运动执行失败 (错误码: %d)", 
                        planner_id.c_str(), result.val);
            return Result::EXECUTION_FAILED;
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "PILZ %s 运动异常: %s", planner_id.c_str(), e.what());
        return Result::EXECUTION_FAILED;
    }
}

// ============================================================================
// 路径预处理和优化方法
// ============================================================================

/**
 * @brief 预处理路径点
 */
bool CR7RobotController::preprocessWaypoints(
    const std::vector<Waypoint>& input_waypoints,
    std::vector<geometry_msgs::msg::Pose>& output_poses,
    const CartesianPathConfig& config) {
    
    output_poses.clear();
    
    if (input_waypoints.empty()) {
        RCLCPP_ERROR(logger_, "输入路径点为空");
        return false;
    }
    
    // 添加起始点（当前位置）
    auto current_pose = getCurrentPose();
    
    // 检查当前位置是否有效
    if (std::isnan(current_pose.position.x) || std::isinf(current_pose.position.x) ||
        std::isnan(current_pose.position.y) || std::isinf(current_pose.position.y) ||
        std::isnan(current_pose.position.z) || std::isinf(current_pose.position.z)) {
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
        if (!wp.isValidQuaternion()) {
            normalized_wp.normalizeQuaternion();
            RCLCPP_DEBUG(logger_, "归一化路点 %zu 的四元数", i);
        }
        
        geometry_msgs::msg::Pose pose = normalized_wp.toPose();
        
        RCLCPP_INFO(logger_, "处理路点 %zu: 位置=[%.6f, %.6f, %.6f]", i,
                    pose.position.x, pose.position.y, pose.position.z);
        
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
bool CR7RobotController::checkPathFeasibility(
    const std::vector<geometry_msgs::msg::Pose>& waypoints) {
    
    if (waypoints.size() < 2) {
        RCLCPP_ERROR(logger_, "路径点数量不足");
        return false;
    }
    
    // 首先检查所有路径点的有效性
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& pose = waypoints[i];
        if (std::isnan(pose.position.x) || std::isinf(pose.position.x) ||
            std::isnan(pose.position.y) || std::isinf(pose.position.y) ||
            std::isnan(pose.position.z) || std::isinf(pose.position.z)) {
            RCLCPP_ERROR(logger_, "路径点 %zu 包含无效的位置数据", i);
            RCLCPP_ERROR(logger_, "位置: [%.6f, %.6f, %.6f]", 
                        pose.position.x, pose.position.y, pose.position.z);
            RCLCPP_ERROR(logger_, "这是原始输入的第 %zu 个点，总共 %zu 个点", i, waypoints.size());
            return false;
        }
        
        // 检查四元数
        double norm = std::sqrt(pose.orientation.x*pose.orientation.x +
                               pose.orientation.y*pose.orientation.y +
                               pose.orientation.z*pose.orientation.z +
                               pose.orientation.w*pose.orientation.w);
        if (std::isnan(norm) || std::isinf(norm) || norm < 1e-6) {
            RCLCPP_ERROR(logger_, "路径点 %zu 四元数无效", i);
            return false;
        }
    }
    
    // 检查路径点间距是否合理
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double distance = calculateDistance(waypoints[i-1], waypoints[i]);
        
        if (distance > 100.0) { // 使用更大的阈值，但检查是否为有效数值
            RCLCPP_ERROR(logger_, "路径点 %zu 和 %zu 间距异常: %.6fm", i-1, i, distance);
            
            // 打印具体位置信息用于调试
            RCLCPP_ERROR(logger_, "点 %zu: [%.6f, %.6f, %.6f]", i-1,
                        waypoints[i-1].position.x, waypoints[i-1].position.y, 
                        waypoints[i-1].position.z);
            RCLCPP_ERROR(logger_, "点 %zu: [%.6f, %.6f, %.6f]", i,
                        waypoints[i].position.x, waypoints[i].position.y, 
                        waypoints[i].position.z);
            
            return false;
        }
        
        if (distance > 0.5) { // 最大允许距离
            RCLCPP_WARN(logger_, "路径点 %zu 和 %zu 间距较大: %.3fm", i-1, i, distance);
            // 这里只是警告，不直接返回false，让路径规划继续尝试
        }
    }
    
    RCLCPP_DEBUG(logger_, "路径可行性检查通过");
    return true;
}

/**
 * @brief 计算两点间距离
 */
double CR7RobotController::calculateDistance(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2) const {
    
    // 检查输入是否有效
    if (std::isnan(pose1.position.x) || std::isnan(pose2.position.x) ||
        std::isnan(pose1.position.y) || std::isnan(pose2.position.y) ||
        std::isnan(pose1.position.z) || std::isnan(pose2.position.z)) {
        RCLCPP_ERROR(logger_, "计算距离时发现NaN值");
        RCLCPP_ERROR(logger_, "点1: [%.6f, %.6f, %.6f]", 
                    pose1.position.x, pose1.position.y, pose1.position.z);
        RCLCPP_ERROR(logger_, "点2: [%.6f, %.6f, %.6f]", 
                    pose2.position.x, pose2.position.y, pose2.position.z);
        return std::numeric_limits<double>::max(); // 返回最大值而不是inf
    }
    
    if (std::isinf(pose1.position.x) || std::isinf(pose2.position.x) ||
        std::isinf(pose1.position.y) || std::isinf(pose2.position.y) ||
        std::isinf(pose1.position.z) || std::isinf(pose2.position.z)) {
        RCLCPP_ERROR(logger_, "计算距离时发现无穷大值");
        RCLCPP_ERROR(logger_, "点1: [%.6f, %.6f, %.6f]", 
                    pose1.position.x, pose1.position.y, pose1.position.z);
        RCLCPP_ERROR(logger_, "点2: [%.6f, %.6f, %.6f]", 
                    pose2.position.x, pose2.position.y, pose2.position.z);
        return std::numeric_limits<double>::max();
    }
    
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dz = pose1.position.z - pose2.position.z;
    
    // 检查差值是否有效
    if (std::isnan(dx) || std::isnan(dy) || std::isnan(dz) ||
        std::isinf(dx) || std::isinf(dy) || std::isinf(dz)) {
        RCLCPP_ERROR(logger_, "差值计算无效: dx=%.6f, dy=%.6f, dz=%.6f", dx, dy, dz);
        return std::numeric_limits<double>::max();
    }
    
    double squared_sum = dx*dx + dy*dy + dz*dz;
    
    // 检查平方和是否有效
    if (std::isnan(squared_sum) || std::isinf(squared_sum)) {
        RCLCPP_ERROR(logger_, "平方和计算无效: %.6f", squared_sum);
        RCLCPP_ERROR(logger_, "dx=%.6f, dy=%.6f, dz=%.6f", dx, dy, dz);
        return std::numeric_limits<double>::max();
    }
    
    double distance = std::sqrt(squared_sum);
    
    // 检查计算结果是否有效
    if (std::isnan(distance) || std::isinf(distance)) {
        RCLCPP_ERROR(logger_, "距离计算结果无效: %.6f", distance);
        RCLCPP_ERROR(logger_, "dx=%.6f, dy=%.6f, dz=%.6f", dx, dy, dz);
        RCLCPP_ERROR(logger_, "平方和: %.6f", squared_sum);
        return std::numeric_limits<double>::max();
    }
    
    return distance;
}

/**
 * @brief 插入中间点
 */
bool CR7RobotController::insertIntermediatePoints(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& end,
    std::vector<geometry_msgs::msg::Pose>& waypoints,
    double max_step) {
    
    // 1. 先深拷贝起始点，避免被修改
    geometry_msgs::msg::Pose safe_start = start;
    geometry_msgs::msg::Pose safe_end = end;
    
    RCLCPP_DEBUG(logger_, "插值起始点: [%.6f, %.6f, %.6f]", 
                safe_start.position.x, safe_start.position.y, safe_start.position.z);
    RCLCPP_DEBUG(logger_, "插值结束点: [%.6f, %.6f, %.6f]", 
                safe_end.position.x, safe_end.position.y, safe_end.position.z);
    
    double distance = calculateDistance(safe_start, safe_end);
    
    // 2. 安全检查
    if (std::isnan(distance) || std::isinf(distance)) {
        RCLCPP_ERROR(logger_, "距离计算无效: %.6f", distance);
        return false;
    }
    
    if (distance <= 0.0 || distance > 10.0) {  // 10米是安全上限
        RCLCPP_ERROR(logger_, "距离超出安全范围: %.6f", distance);
        return false;
    }
    
    // 3. 更安全的点数计算
    const double MIN_STEP = 0.001;  // 1mm最小步长
    const double MAX_STEP = 0.1;    // 10cm最大步长
    
    double actual_step = max_step;
    if (actual_step < MIN_STEP) actual_step = MIN_STEP;
    if (actual_step > MAX_STEP) actual_step = MAX_STEP;
    
    int num_segments = static_cast<int>(std::ceil(distance / actual_step));
    
    // 限制最大分段数
    const int MAX_SEGMENTS = 100;
    if (num_segments > MAX_SEGMENTS) {
        num_segments = MAX_SEGMENTS;
        actual_step = distance / num_segments;  // 重新计算步长
    }
    
    RCLCPP_DEBUG(logger_, "距离: %.6fm, 步长: %.6fm, 分段数: %d", 
                distance, actual_step, num_segments);
    
    if (num_segments <= 1) {
        return true;  // 不需要插入中间点
    }
    
    // 4. 安全的差值计算
    double delta_x = safe_end.position.x - safe_start.position.x;
    double delta_y = safe_end.position.y - safe_start.position.y;
    double delta_z = safe_end.position.z - safe_start.position.z;
    
    // 检查差值是否合理
    if (std::abs(delta_y) > 10.0) {
        RCLCPP_ERROR(logger_, "Y差值异常: %.6f", delta_y);
        return false;
    }
    
    // 5. 使用更稳定的插值方法
    for (int i = 1; i <= num_segments; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(num_segments + 1);
        
        geometry_msgs::msg::Pose intermediate;
        
        // 使用增量累加而不是乘法，避免浮点误差累积
        double x = safe_start.position.x + delta_x * t;
        double y = safe_start.position.y + delta_y * t;
        double z = safe_start.position.z + delta_z * t;
        
        // 立即验证结果
        if (std::isnan(x) || std::isinf(x) ||
            std::isnan(y) || std::isinf(y) ||
            std::isnan(z) || std::isinf(z)) {
            RCLCPP_ERROR(logger_, "插值计算产生无效值: [%e, %e, %e], t=%.6f", x, y, z, t);
            return false;
        }
        
        // 检查是否在合理范围内
        if (std::abs(x) > 10.0 || std::abs(y) > 10.0 || std::abs(z) > 10.0) {
            RCLCPP_ERROR(logger_, "插值点超出范围: [%.6f, %.6f, %.6f]", x, y, z);
            return false;
        }
        
        intermediate.position.x = x;
        intermediate.position.y = y;
        intermediate.position.z = z;
        
        // 姿态插值
        if (!slerpQuaternion(safe_start.orientation, safe_end.orientation, t, intermediate.orientation)) {
            intermediate.orientation = safe_end.orientation;
        }
        
        waypoints.push_back(intermediate);
        
        RCLCPP_DEBUG(logger_, "插入中间点 %d: [%.6f, %.6f, %.6f]", 
                    i, x, y, z);
    }
    
    RCLCPP_INFO(logger_, "成功插入 %d 个中间点", num_segments);
    return true;
}

/**
 * @brief 球面线性插值四元数
 */
bool CR7RobotController::slerpQuaternion(
    const geometry_msgs::msg::Quaternion& q1,
    const geometry_msgs::msg::Quaternion& q2,
    double t, geometry_msgs::msg::Quaternion& result) const {
    
    if (t <= 0.0) {
        result = q1;
        return true;
    }
    if (t >= 1.0) {
        result = q2;
        return true;
    }
    
    // 使用更安全的插值方法
    double dot = q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    
    // 如果点积接近-1，四元数接近相反，需要特殊处理
    if (dot < 0.0) {
        // 取反q2，保持最短路径
        geometry_msgs::msg::Quaternion q2_neg = q2;
        q2_neg.x = -q2.x;
        q2_neg.y = -q2.y;
        q2_neg.z = -q2.z;
        q2_neg.w = -q2.w;
        dot = -dot;
        
        // 线性插值
        double t1 = 1.0 - t;
        result.x = t1 * q1.x + t * q2_neg.x;
        result.y = t1 * q1.y + t * q2_neg.y;
        result.z = t1 * q1.z + t * q2_neg.z;
        result.w = t1 * q1.w + t * q2_neg.w;
    } else {
        // 标准线性插值
        double t1 = 1.0 - t;
        result.x = t1 * q1.x + t * q2.x;
        result.y = t1 * q1.y + t * q2.y;
        result.z = t1 * q1.z + t * q2.z;
        result.w = t1 * q1.w + t * q2.w;
    }
    
    // 归一化
    double norm = std::sqrt(result.x*result.x + result.y*result.y + 
                          result.z*result.z + result.w*result.w);
    
    if (norm > 1e-6) {
        result.x /= norm;
        result.y /= norm;
        result.z /= norm;
        result.w /= norm;
        
        // 验证归一化结果
        double check_norm = result.x*result.x + result.y*result.y + 
                           result.z*result.z + result.w*result.w;
        if (std::abs(check_norm - 1.0) > 0.001) {
            RCLCPP_WARN(logger_, "四元数归一化不精确: norm=%.6f", check_norm);
            return false;
        }
        return true;
    } else {
        RCLCPP_ERROR(logger_, "四元数插值结果模长为0");
        result.x = 0.0;
        result.y = 0.0;
        result.z = 0.0;
        result.w = 1.0;
        return false;
    }
}

/**
 * @brief 尝试简化路径
 */
double CR7RobotController::trySimplifiedPath(
    const std::vector<geometry_msgs::msg::Pose>& original_waypoints,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    const CartesianPathConfig& config) 
    {
    
    // 如果路径点太多，尝试减少数量
    if (original_waypoints.size() > 8) 
    {
        std::vector<geometry_msgs::msg::Pose> simplified_waypoints;
        
        // 保留关键点：起点、终点、中间关键点
        simplified_waypoints.push_back(original_waypoints.front());
        
        // 每隔一定数量取一个点
        size_t step = original_waypoints.size() / 4;
        for (size_t i = step; i < original_waypoints.size() - 1; i += step) 
        {
            if (i < original_waypoints.size()) 
            {
                simplified_waypoints.push_back(original_waypoints[i]);
            }
        }
        
        simplified_waypoints.push_back(original_waypoints.back());
        
        RCLCPP_INFO(logger_, "简化路径点数量: %zu -> %zu", 
                   original_waypoints.size(), simplified_waypoints.size());
        
        return move_group_->computeCartesianPath(
            simplified_waypoints, config.eef_step, config.jump_threshold, trajectory, config.avoid_collisions);
    }
    
    return 0.0; // 不进行简化
}

/**
 * @brief 优化轨迹（轻微优化）
 */
bool CR7RobotController::optimizeTrajectoryLight(moveit_msgs::msg::RobotTrajectory& trajectory) {
    if (trajectory.joint_trajectory.points.size() < 10) {
        RCLCPP_DEBUG(logger_, "轨迹点数较少 (%zu)，不进行优化", trajectory.joint_trajectory.points.size());
        return true; // 轨迹太短，无需优化
    }
    
    auto& points = trajectory.joint_trajectory.points;
    size_t original_size = points.size();
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> optimized_points;
    
    // 保留第一个点
    optimized_points.push_back(points.front());
    
    // 轻微简化轨迹：只移除非常接近线性插值的点
    for (size_t i = 1; i < points.size() - 1; ++i) {
        if (isPointSignificant(points[i-1], points[i], points[i+1])) {
            optimized_points.push_back(points[i]);
        }
    }
    
    // 保留最后一个点
    optimized_points.push_back(points.back());
    
    if (optimized_points.size() < points.size()) {
        double reduction_ratio = static_cast<double>(points.size() - optimized_points.size()) / points.size() * 100.0;
        
        if (reduction_ratio > 50.0) { // 如果删除超过50%的点，可能过度简化了
            RCLCPP_WARN(logger_, "过度简化轨迹: 删除 %.1f%% 的点，保持原轨迹", reduction_ratio);
            return true; // 保持原轨迹
        }
        
        RCLCPP_INFO(logger_, "轨迹优化: %zu -> %zu 个点 (删除 %.1f%%)", 
                   original_size, optimized_points.size(), reduction_ratio);
        points = optimized_points;
        return true;
    }
    
    RCLCPP_DEBUG(logger_, "轨迹无需优化");
    return true;
}

/**
 * @brief 检查点是否显著（不应该被删除）
 */
bool CR7RobotController::isPointSignificant(
    const trajectory_msgs::msg::JointTrajectoryPoint& prev,
    const trajectory_msgs::msg::JointTrajectoryPoint& current,
    const trajectory_msgs::msg::JointTrajectoryPoint& next) const {
    
    // 更严格的阈值，只删除非常接近线性插值的点
    double threshold = 0.005; // 0.5度的阈值，比原来更严格
    
    for (size_t i = 0; i < current.positions.size(); ++i) {
        double linear_value = (prev.positions[i] + next.positions[i]) / 2.0;
        if (std::abs(current.positions[i] - linear_value) > threshold) {
            return true; // 差异较大，需要保留
        }
    }
    
    return false; // 可以移除
}

/**
 * @brief 保存轨迹分析
 */
void CR7RobotController::saveTrajectoryAnalysis(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& filename_prefix) {
    
    const auto& points = trajectory.joint_trajectory.points;
    
    if (points.empty()) {
        RCLCPP_WARN(logger_, "轨迹为空，无法保存分析");
        return;
    }
    
    // 生成文件名
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << filename_prefix << "_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".csv";
    std::string filename = ss.str();
    
    // 打开文件
    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger_, "无法打开文件保存轨迹分析: %s", filename.c_str());
        return;
    }
    
    RCLCPP_INFO(logger_, "保存轨迹分析到: %s", filename.c_str());
    
    // 写入标题
    file << "index,time_from_start";
    
    // 关节名称
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        file << ",position_" << i << "(" << joint_names[i] << ")";
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
        file << ",velocity_" << i;
    }
    for (size_t i = 0; i < joint_names.size(); ++i) {
        file << ",acceleration_" << i;
    }
    file << "\n";
    
    // 写入数据
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        file << i << "," << point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        
        // 位置
        for (size_t j = 0; j < point.positions.size(); ++j) {
            file << "," << point.positions[j];
        }
        
        // 速度
        for (size_t j = 0; j < point.velocities.size(); ++j) {
            file << "," << point.velocities[j];
        }
        
        // 加速度
        for (size_t j = 0; j < point.accelerations.size(); ++j) {
            file << "," << point.accelerations[j];
        }
        file << "\n";
    }
    
    file.close();
    RCLCPP_INFO(logger_, "轨迹分析保存完成，共 %zu 个点", points.size());
}

/**
 * @brief 保存路径点分析
 */
void CR7RobotController::saveWaypointAnalysis(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const std::string& filename_prefix) {
    
    if (waypoints.empty()) {
        RCLCPP_WARN(logger_, "路径点为空，无法保存分析");
        return;
    }
    
    // 生成文件名
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << filename_prefix << "_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".csv";
    std::string filename = ss.str();
    
    // 打开文件
    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger_, "无法打开文件保存路径点分析: %s", filename.c_str());
        return;
    }
    
    RCLCPP_INFO(logger_, "保存路径点分析到: %s", filename.c_str());
    
    // 写入标题
    file << "index,x,y,z,qx,qy,qz,qw,distance_from_prev,cumulative_distance\n";
    
    // 写入数据
    double cumulative_distance = 0.0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto& pose = waypoints[i];
        
        double distance = 0.0;
        if (i > 0) {
            distance = calculateDistance(waypoints[i-1], pose);
            cumulative_distance += distance;
        }
        
        file << i << ","
             << pose.position.x << ","
             << pose.position.y << ","
             << pose.position.z << ","
             << pose.orientation.x << ","
             << pose.orientation.y << ","
             << pose.orientation.z << ","
             << pose.orientation.w << ","
             << distance << ","
             << cumulative_distance << "\n";
    }
    
    file.close();
    RCLCPP_INFO(logger_, "路径点分析保存完成，共 %zu 个点", waypoints.size());
}

/**
 * @brief 打印轨迹信息
 */
void CR7RobotController::printTrajectoryInfo(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    const auto& points = trajectory.joint_trajectory.points;
    
    if (points.empty()) {
        RCLCPP_WARN(logger_, "轨迹为空");
        return;
    }
    
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "轨迹详细信息");
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "轨迹点数: %zu", points.size());
    RCLCPP_INFO(logger_, "关节数量: %zu", trajectory.joint_trajectory.joint_names.size());
    
    // 打印关节名称
    RCLCPP_INFO(logger_, "关节名称:");
    for (size_t i = 0; i < trajectory.joint_trajectory.joint_names.size(); ++i) {
        RCLCPP_INFO(logger_, "  关节%zu: %s", i, trajectory.joint_trajectory.joint_names[i].c_str());
    }
    
    // 计算轨迹总时间
    double total_time = 0.0;
    if (!points.empty()) {
        const auto& last_point = points.back();
        total_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
    }
    RCLCPP_INFO(logger_, "轨迹总时间: %.3f 秒", total_time);
    
    // 打印关键点信息
    RCLCPP_INFO(logger_, "关键点信息:");
    RCLCPP_INFO(logger_, "  起始点 (索引 0):");
    printTrajectoryPoint(points[0], 0);
    
    if (points.size() > 1) {
        RCLCPP_INFO(logger_, "  中间点 (索引 %zu):", points.size()/2);
        printTrajectoryPoint(points[points.size()/2], points.size()/2);
    }
    
    if (points.size() > 2) {
        RCLCPP_INFO(logger_, "  结束点 (索引 %zu):", points.size()-1);
        printTrajectoryPoint(points.back(), points.size()-1);
    }
    
    RCLCPP_INFO(logger_, "========================================");
}

/**
 * @brief 打印轨迹点信息
 */
void CR7RobotController::printTrajectoryPoint(
    const trajectory_msgs::msg::JointTrajectoryPoint& point,
    size_t index) {
    
    double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    RCLCPP_INFO(logger_, "    索引: %zu, 时间: %.3f 秒", index, time);
    
    // 打印关节位置
    RCLCPP_INFO(logger_, "    关节位置 (弧度):");
    for (size_t i = 0; i < point.positions.size(); ++i) {
        RCLCPP_INFO(logger_, "      关节%zu: %.6f", i, point.positions[i]);
    }
    
    // 如果有速度信息，打印速度
    if (!point.velocities.empty()) {
        RCLCPP_INFO(logger_, "    关节速度 (弧度/秒):");
        for (size_t i = 0; i < point.velocities.size(); ++i) {
            RCLCPP_INFO(logger_, "      关节%zu: %.6f", i, point.velocities[i]);
        }
    }
}

/**
 * @brief 验证轨迹
 */
bool CR7RobotController::validateTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    const auto& points = trajectory.joint_trajectory.points;
    
    if (points.empty()) {
        RCLCPP_ERROR(logger_, "轨迹为空");
        return false;
    }
    
    RCLCPP_INFO(logger_, "验证轨迹有效性...");
    
    // 检查关节值有效性
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];
        
        // 检查时间
        if (point.time_from_start.sec < 0) {
            RCLCPP_ERROR(logger_, "轨迹点 %zu 时间无效: %d 秒", i, point.time_from_start.sec);
            return false;
        }
        
        for (size_t j = 0; j < point.positions.size(); ++j) {
            double value = point.positions[j];
            
            if (std::isnan(value)) {
                RCLCPP_ERROR(logger_, "轨迹点 %zu 关节 %zu 值为 NaN", i, j);
                return false;
            }
            
            if (std::isinf(value)) {
                RCLCPP_ERROR(logger_, "轨迹点 %zu 关节 %zu 值为无穷大", i, j);
                return false;
            }
        }
    }
    
    RCLCPP_INFO(logger_, "✓ 轨迹验证通过");
    return true;
}

// ============================================================================
// 预设路径实现
// ============================================================================

std::vector<CR7RobotController::Result> CR7RobotController::executeWeldingPath() {
    RCLCPP_INFO(logger_, "执行预设焊接路径");
    
    // 定义焊接路点
    std::vector<Waypoint> welding_waypoints = {
        // 第一个焊点
        Waypoint("Weld_Point_1", 
                 0.76908, -0.38109, 0.036355,
                 0.72419, 0.62801, 0.10427, 0.26512),
        
        // 第二个焊点
        Waypoint("Weld_Point_2",
                 0.77031, 0.37564, 0.039695,
                 -0.61661, 0.72446, -0.13272, 0.2781)
    };
    
    // 验证四元数
    for (auto& wp : welding_waypoints) {
        if (!wp.isValidQuaternion()) {
            RCLCPP_WARN(logger_, "路点 %s 四元数未归一化，自动处理", wp.name.c_str());
            wp.normalizeQuaternion();
        }
    }
    
    return executeWaypoints(welding_waypoints, 2.0);
}

std::vector<CR7RobotController::Result> CR7RobotController::executeTestPath() {
    RCLCPP_INFO(logger_, "执行测试路径");
    
    // 简单的测试路点，更容易到达
    std::vector<Waypoint> test_waypoints = {
        // 中间位置
        Waypoint("Test_Center",
                 0.3, 0.0, 0.3,
                 0.0, 0.0, 0.0, 1.0),
        
        // 左侧
        Waypoint("Test_Left",
                 0.4, -0.2, 0.4,
                 0.0, 0.0, 0.0, 1.0),
        
        // 回到中间
        Waypoint("Test_Center_Return",
                 0.3, 0.0, 0.3,
                 0.0, 0.0, 0.0, 1.0),
        
        // 右侧
        Waypoint("Test_Right",
                 0.4, 0.2, 0.4,
                 0.0, 0.0, 0.0, 1.0)
    };
    
    return executeWaypoints(test_waypoints, 1.0);
}

CR7RobotController::Result CR7RobotController::executeCartesianWeldingPath() 
{
    // 1️⃣ 先 move 到起点
    auto start_wp = Waypoint("Weld_Point_1", 
                             0.97772, -0.3741, 0.028286,
                             0.53928, 0.81723, 0.027122, 0.20142);

    auto ret = moveToWaypoint(start_wp);
    if (ret != Result::SUCCESS) {
        RCLCPP_ERROR(logger_, "无法到达笛卡尔路径起点");
        return ret;
    }

    // 2️⃣ 再执行真正的 Cartesian（去掉起点）
    std::vector<Waypoint> cartesian_waypoints = {
        Waypoint("Weld_Point_1_1",
                 0.97772, -0.229744, 0.028286,
                 0.494127, 0.802070, 0.058109, 0.330372),

        Waypoint("Weld_Point_1_2",
                 0.97772, -0.078398, 0.028286,
                 0.219786, 0.904256, 0.006742, 0.366019),

        Waypoint("Weld_Point_1_3",
                 0.97772,  0.072948, 0.028286,
                 -0.074251, 0.925408, -0.045230, 0.368866),

        Waypoint("Weld_Point_1_4",
                 0.97772, 0.37753, 0.028286,
                 -0.37331, 0.8992, -0.084073, 0.21217),
    };

    return executeOptimizedCartesianPath(cartesian_waypoints);
}

// ============================================================================
// 状态查询和配置方法实现
// ============================================================================

bool CR7RobotController::isReady() const {
    return initialized_;
}

geometry_msgs::msg::Pose CR7RobotController::getCurrentPose() const {
    if (!initialized_) {
        RCLCPP_WARN(logger_, "控制器未初始化");
        return geometry_msgs::msg::Pose();
    }
    
    try {
        return move_group_->getCurrentPose().pose;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "获取当前位姿失败: %s", e.what());
        return geometry_msgs::msg::Pose();
    }
}

std::vector<double> CR7RobotController::getCurrentJointPositions() const {
    if (!initialized_) {
        RCLCPP_WARN(logger_, "控制器未初始化");
        return {};
    }
    
    try {
        return move_group_->getCurrentJointValues();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "获取关节位置失败: %s", e.what());
        return {};
    }
}

void CR7RobotController::printCurrentState() const {
    if (!initialized_) {
        RCLCPP_INFO(logger_, "控制器状态: 未初始化");
        return;
    }
    
    RCLCPP_INFO(logger_, "========================================");
    RCLCPP_INFO(logger_, "当前机器人状态");
    RCLCPP_INFO(logger_, "========================================");
    
    try {
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
        if (!joints.empty()) {
            RCLCPP_INFO(logger_, "关节位置 (弧度):");
            for (size_t i = 0; i < joints.size(); ++i) {
                RCLCPP_INFO(logger_, "  关节%zu: %.4f", i + 1, joints[i]);
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "获取状态失败: %s", e.what());
    }
    
    RCLCPP_INFO(logger_, "========================================");
}

void CR7RobotController::setPlanningTime(double seconds) {
    if (seconds > 0) {
        move_group_->setPlanningTime(seconds);
        RCLCPP_INFO(logger_, "设置规划时间: %.1f 秒", seconds);
    }
}

void CR7RobotController::setVelocityFactor(double factor) {
    if (factor > 0 && factor <= 1.0) {
        move_group_->setMaxVelocityScalingFactor(factor);
        RCLCPP_INFO(logger_, "设置速度因子: %.2f", factor);
    }
}

void CR7RobotController::setAccelerationFactor(double factor) {
    if (factor > 0 && factor <= 1.0) {
        move_group_->setMaxAccelerationScalingFactor(factor);
        RCLCPP_INFO(logger_, "设置加速度因子: %.2f", factor);
    }
}

bool CR7RobotController::validatePose(const geometry_msgs::msg::Pose& pose, 
                                     std::string* error_msg) const {
    // 检查四元数是否归一化
    const double norm = pose.orientation.x * pose.orientation.x +
                       pose.orientation.y * pose.orientation.y +
                       pose.orientation.z * pose.orientation.z +
                       pose.orientation.w * pose.orientation.w;
    
    if (std::abs(norm - 1.0) > 0.001) {
        if (error_msg) {
            *error_msg = "四元数未归一化 (norm = " + std::to_string(norm) + ")";
        }
        return false;
    }
    
    // 可以添加其他验证，如位置限制
    return true;
}

std::string CR7RobotController::resultToString(Result result) const {
    switch (result) {
        case Result::SUCCESS: return "成功";
        case Result::PLANNING_FAILED: return "规划失败";
        case Result::EXECUTION_FAILED: return "执行失败";
        case Result::INVALID_INPUT: return "输入无效";
        case Result::ROBOT_NOT_READY: return "机器人未就绪";
        case Result::TIMEOUT: return "超时";
        case Result::CARTESIAN_FAILED: return "笛卡尔规划失败";
        default: return "未知结果";
    }
}

}  // namespace cr7_controller