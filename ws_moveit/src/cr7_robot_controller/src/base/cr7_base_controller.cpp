/**
 * @file cr7_base_controller.cpp
 * @brief CR7机器人基础控制器实现文件
 * 
 * 实现CR7BaseController类的所有方法，包括：
 * 1. 机器人状态管理
 * 2. 规划器管理和协调
 * 3. 状态查询和监控
 * 4. 路点管理
 */

#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"
#include "cr7_robot_controller/utils/pose_utils.hpp"

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// Waypoint 方法实现
// ============================================================================

/**
 * @brief 转换为ROS PoseStamped消息
 * @return geometry_msgs::msg::PoseStamped ROS位姿消息
 */  
geometry_msgs::msg::PoseStamped Waypoint::toPoseStamped() const 
{  // 新增方法
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.pose = toPose();
    return pose_stamped;
}


/**
 * @brief 转换为ROS Pose消息
 * @return geometry_msgs::msg::Pose ROS位姿消息
 */
geometry_msgs::msg::Pose Waypoint::toPose() const 
{
    geometry_msgs::msg::Pose pose; ;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
}

/**
 * @brief 验证四元数是否有效
 * @return bool 如果四元数是单位四元数则返回true
 */
bool Waypoint::isValidQuaternion() const 
{
    const double norm = qx*qx + qy*qy + qz*qz + qw*qw;
    return std::abs(norm - 1.0) < 0.001;
}

/**
 * @brief 归一化四元数
 * 
 * 如果四元数不是单位四元数，将其归一化
 */
void Waypoint::normalizeQuaternion() 
{
    const double norm = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm > 1e-6) 
    {
        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;
    } 
    else 
    {
        // 如果四元数为零，设置为单位四元数
        qx = 0.0;
        qy = 0.0;
        qz = 0.0;
        qw = 1.0;
    }
}

// ============================================================================
// CR7BaseController 方法实现 （公开方法实现）
// ============================================================================

/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param planning_group MoveIt规划组名称
 */
CR7BaseController::CR7BaseController(rclcpp::Node::SharedPtr node, 
                                   const std::string& planning_group)
    : node_(node)
    , logger_(node->get_logger())
    , initialized_(false) 
{
    
    RCLCPP_INFO(logger_, "创建CR7基础机器人控制器");
    RCLCPP_INFO(logger_, "规划组: %s", planning_group.c_str());

    try 
    {
        // 创建MoveGroup接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, planning_group);

        RCLCPP_INFO(logger_, "MoveGroup接口创建成功");
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_FATAL(logger_, "创建MoveGroup接口失败: %s", e.what());
        throw;
    }
}

/**
 * @brief 初始化控制器
 * @param timeout_sec 初始化超时时间(秒)
 * @return bool 初始化成功返回true
 * 
 * 等待机器人状态就绪，初始化MoveIt接口
 */
bool CR7BaseController::initialize(double timeout_sec) 
{
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
    
    RCLCPP_INFO(logger_, "机器人控制器初始化完成");
    RCLCPP_INFO(logger_, "规划时间: %.1f秒", move_group_->getPlanningTime());
    
    initialized_ = true;
    return true;
}


/**
 * @brief 设置OMPL规划器
 * @param ompl_planner OMPL规划器实例
 */
void CR7BaseController::setOMPLPlanner(std::shared_ptr<CR7OMPLPlanner> ompl_planner)
{
    ompl_planner_ = ompl_planner;
    RCLCPP_INFO(logger_, "OMPL规划器设置成功");
}

/**
 * @brief 获取OMPL规划器
 * @return std::shared_ptr<CR7OMPLPlanner> OMPL规划器实例
 */
std::shared_ptr<CR7OMPLPlanner> CR7BaseController::getOMPLPlanner() const
{
    return ompl_planner_;
}

/**
 * @brief 检查规划器是否就绪
 * @return bool 规划器就绪返回true
 */
bool CR7BaseController::isPlannerReady() const
{
    return ompl_planner_ != nullptr;
}


// ============================================================================
// 状态查询和配置方法
// ============================================================================

/**
 * @brief 检查机器人是否就绪
 * @return bool 机器人就绪返回true
 */
bool CR7BaseController::isReady() const 
{
    return initialized_;
}

/**
 * @brief 获取当前末端执行器位姿
 * @return geometry_msgs::msg::Pose 当前位姿
 */
geometry_msgs::msg::Pose CR7BaseController::getCurrentPose() const 
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
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

/**
 * @brief 获取当前关节位置
 * @return std::vector<double> 关节位置(弧度)
 */
std::vector<double> CR7BaseController::getCurrentJointPositions() const 
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return std::vector<double>();
    }
    
    try 
    {
        return move_group_->getCurrentJointValues();
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "获取当前关节位置失败: %s", e.what());
        return std::vector<double>();
    }
}

/**
 * @brief 打印当前状态
 */
void CR7BaseController::printCurrentState() const 
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return;
    }
    
    try 
    {
        auto current_pose = getCurrentPose();
        auto joint_positions = getCurrentJointPositions();
        
        RCLCPP_INFO(logger_, "=====================================");
        RCLCPP_INFO(logger_, "当前机器人状态");
        RCLCPP_INFO(logger_, "=====================================");
        RCLCPP_INFO(logger_, "末端执行器位置: [%.3f, %.3f, %.3f]", 
                    current_pose.position.x, current_pose.position.y, current_pose.position.z);
        RCLCPP_INFO(logger_, "末端执行器姿态: [%.3f, %.3f, %.3f, %.3f]",
                    current_pose.orientation.x, current_pose.orientation.y,
                    current_pose.orientation.z, current_pose.orientation.w);
        
        if (!joint_positions.empty()) 
        {
            RCLCPP_INFO(logger_, "关节位置:");
            const auto& joint_names = move_group_->getJointNames();
            for (size_t i = 0; i < joint_positions.size() && i < joint_names.size(); ++i) 
            {
                RCLCPP_INFO(logger_, "  %s: %.3f rad", joint_names[i].c_str(), joint_positions[i]);
            }
        }
        
        RCLCPP_INFO(logger_, "=====================================");
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "打印当前状态失败: %s", e.what());
    }
}



/**
 * @brief 结果转字符串（公开方法）
 * @param result 规划结果
 * @return std::string 结果描述
 */
std::string CR7BaseController::resultToString(Result result) const {
    return utils::PoseUtils::resultToString(result);
}

// ============================================================================
// CR7BaseController 方法实现 （私有方法实现）
// ============================================================================

/**
 * @brief 等待机器人状态就绪
 */
bool CR7BaseController::waitForRobotState(double timeout_sec) 
{
    RCLCPP_INFO(logger_, "等待机器人状态 (timeout = %.2f s)...", timeout_sec);

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









}  // namespace cr7_controller
