/**
 * @file cr7_robot_controller.cpp
 * @brief CR7机器人控制器实现文件
 * 
 * 实现CR7RobotController类的所有方法
 */

#include "cr7_robot_controller/cr7_robot_controller.hpp"
#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>

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
    move_group_->setPlanningTime(5.0);           // 规划时间5秒
    move_group_->setNumPlanningAttempts(50);      // 尝试50次
    move_group_->setMaxVelocityScalingFactor(0.5);  // 最大速度50%
    move_group_->setMaxAccelerationScalingFactor(0.3);  // 最大加速度30%
    
    // 3. 设置目标容差
    move_group_->setGoalPositionTolerance(0.001);    // 1mm
    move_group_->setGoalOrientationTolerance(0.01);  // 约0.57度
    
    RCLCPP_INFO(logger_, "机器人控制器初始化完成");
    RCLCPP_INFO(logger_, "规划时间: %.1f秒", move_group_->getPlanningTime());
    // RCLCPP_INFO(logger_, "速度因子: %.1f", move_group_->getMaxVelocityScalingFactor());
    
    initialized_ = true;
    return true;
}



// bool CR7RobotController::waitForRobotState(double /*timeout_sec*/)
// {
//     RCLCPP_INFO(logger_, "尝试非阻塞获取机器人状态...");

//     // 1️⃣ 非阻塞获取 MoveIt 内部缓存状态
//     moveit::core::RobotStatePtr current_state =
//         move_group_->getCurrentState();

//     if (!current_state)
//     {
//         RCLCPP_WARN(logger_, "MoveIt 尚未缓存任何 RobotState");
//         return false;
//     }

//     // 2️⃣ 通过 RobotModel 校验关节是否合法（这是唯一稳定方式）
//     const moveit::core::RobotModelConstPtr& robot_model =
//         move_group_->getRobotModel();

//     if (!robot_model)
//     {
//         RCLCPP_ERROR(logger_, "RobotModel 为空");
//         return false;
//     }

//     const auto& joint_names = move_group_->getJointNames();
//     if (joint_names.empty())
//     {
//         RCLCPP_WARN(logger_, "MoveGroup 未包含任何关节");
//         return false;
//     }

//     // 3️⃣ 校验每个关节是否存在于模型中
//     for (const auto& name : joint_names)
//     {
//         const moveit::core::JointModel* jm =
//             robot_model->getJointModel(name);

//         if (!jm)
//         {
//             RCLCPP_WARN(
//                 logger_,
//                 "RobotModel 中不存在关节: %s",
//                 name.c_str()
//             );
//             return false;
//         }
//     }

//     // 4️⃣ 现在可以安全读取 joint 值
//     RCLCPP_INFO(logger_, "✓ 成功获取机器人状态（非阻塞）");
//     RCLCPP_INFO(logger_, "关节数量: %zu", joint_names.size());

//     for (const auto& name : joint_names)
//     {
//         double value = current_state->getVariablePosition(name);
//         RCLCPP_DEBUG(logger_, "  %s = %.6f", name.c_str(), value);
//     }

//     return true;
// }


bool CR7RobotController::waitForRobotState(double timeout_sec) 
{
    RCLCPP_INFO(logger_, "等待机器人状态 (timeout = %.2f s)...", timeout_sec);

    // MoveIt2 官方阻塞接口：
    // 会等待 joint_states，直到 CurrentStateMonitor 收到“时间有效”的状态
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
// {
//     RCLCPP_INFO(logger_, "等待机器人状态...");
    
//     const auto start_time = node_->now();
//     int attempts = 0;
    
//     while (rclcpp::ok() && (node_->now() - start_time).seconds() < timeout_sec) {
//         try {
//             // 尝试获取当前状态
//             auto current_state = move_group_->getCurrentState();
//             if (current_state) {
//                 RCLCPP_INFO(logger_, "✓ 收到机器人状态");
                
//                 // 获取关节数量
//                 auto joint_names = move_group_->getJointNames();
//                 RCLCPP_INFO(logger_, "关节数量: %zu", joint_names.size());
                
//                 return true;
//             }
//         } catch (const std::exception& e) {
//             RCLCPP_DEBUG(logger_, "获取状态失败: %s", e.what());
//         }
        
//         // 每5秒打印一次等待信息
//         if (attempts % 10 == 0) {
//             double elapsed = (node_->now() - start_time).seconds();
//             RCLCPP_INFO(logger_, "等待机器人状态... 已等待 %.1f 秒", elapsed);
//         }
        
//         rclcpp::sleep_for(500ms);
//         rclcpp::spin_some(node_);
//         attempts++;
//     }
    
//     RCLCPP_ERROR(logger_, "等待机器人状态超时 (%.1f 秒)", timeout_sec);
//     return false;
// }

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
        default: return "未知结果";
    }
}

}  // namespace cr7_controller