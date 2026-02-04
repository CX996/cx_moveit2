/**
 * @file cr7_base_controller.cpp
 * @brief CR7机器人基础控制器实现文件
 * 
 * 实现CR7BaseController类的所有方法
 */

#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// Waypoint 方法实现
// ============================================================================

/**
 * @brief 转换为ROS PoseStamped消息
 * @return geometry_msgs::msg::PoseStamped ROS位姿消息
 */  
geometry_msgs::msg::PoseStamped Waypoint::toPoseStamped() const {  // 新增方法
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
    
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(logger_, "创建CR7基础机器人控制器");
    RCLCPP_INFO(logger_, "规划组: %s", planning_group.c_str());

    try 
    {
        // 创建MoveGroup接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, planning_group);
        
        move_group_->setPlannerId("RRTstarkConfigDefault");  // 例如使用RRTstark

        // 创建规划场景接口
        planning_scene_interface_ = 
            std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        
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


/**
 * @brief 移动到单个位姿
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return Result 规划结果
 */
CR7BaseController::Result CR7BaseController::moveToPose(const geometry_msgs::msg::Pose& target_pose,
                                                      const std::string& waypoint_name) 
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = move_group_->getPlanningFrame(); // 默认规划坐标系
    pose_stamped.pose = target_pose;
    return moveToPose(pose_stamped, waypoint_name);
}

/**
 * @brief 移动到单个位姿，修改 moveToPose 方法，支持 PoseStamped
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return Result 规划结果
 */
CR7BaseController::Result CR7BaseController::moveToPose(const geometry_msgs::msg::PoseStamped& target_pose,
                                                      const std::string& waypoint_name) 
{
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    // 获取规划坐标系（通常是机器人的基坐标系）
    std::string planning_frame = move_group_->getPlanningFrame();
    
    geometry_msgs::msg::PoseStamped transformed_pose;
    
    // 如果目标坐标系与规划坐标系不同，进行坐标变换
    if (target_pose.header.frame_id != planning_frame) {
        RCLCPP_INFO(logger_, "变换坐标系: %s -> %s", 
                    target_pose.header.frame_id.c_str(), planning_frame.c_str());
        
        if (!transformPose(target_pose, transformed_pose, planning_frame)) {
            RCLCPP_ERROR(logger_, "坐标变换失败");
            return Result::INVALID_INPUT;
        }
    } else {
        transformed_pose = target_pose;
    }
    
    // 使用变换后的位姿继续执行
    return moveToPoseImpl(transformed_pose.pose, waypoint_name);
}


/**
 * @brief 移动到路点
 * @param waypoint 目标路点
 * @return Result 规划结果
 */
CR7BaseController::Result CR7BaseController::moveToWaypoint(const Waypoint& waypoint) {
    // 检查四元数
    if (!waypoint.isValidQuaternion()) {
        RCLCPP_WARN(logger_, "四元数未归一化，自动处理...");
        Waypoint normalized_waypoint = waypoint;
        normalized_waypoint.normalizeQuaternion();
        return moveToPose(normalized_waypoint.toPoseStamped(), waypoint.name);
    }
    
    return moveToPose(waypoint.toPoseStamped(), waypoint.name);
}

/**
 * @brief 执行多路点序列
 * @param waypoints 路点向量
 * @param delay_seconds 路点间延迟(秒)
 * @return std::vector<Result> 每个路点的结果
 */
std::vector<CR7BaseController::Result> CR7BaseController::executeWaypoints(
    const std::vector<Waypoint>& waypoints,
    double delay_seconds) {
    
    std::vector<Result> results;
    
    if (!initialized_) {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        results.push_back(Result::ROBOT_NOT_READY);
        return results;
    }
    
    RCLCPP_INFO(logger_, "=====================================");
    RCLCPP_INFO(logger_, "开始执行 %zu 个路点", waypoints.size());
    RCLCPP_INFO(logger_, "路点间延迟: %.1f 秒", delay_seconds);
    RCLCPP_INFO(logger_, "=====================================");
    
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
    
    RCLCPP_INFO(logger_, "=====================================");
    RCLCPP_INFO(logger_, "路点序列执行完成");
    RCLCPP_INFO(logger_, "成功: %d/%zu", success_count, waypoints.size());
    RCLCPP_INFO(logger_, "=====================================");
    
    return results;
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
 * @brief 设置规划时间
 * @param seconds 规划时间(秒)
 */
void CR7BaseController::setPlanningTime(double seconds) 
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return;
    }
    
    move_group_->setPlanningTime(seconds);
    RCLCPP_INFO(logger_, "设置规划时间: %.1f秒", seconds);
}

/**
 * @brief 设置速度因子
 * @param factor 速度因子(0.0-1.0)
 */
void CR7BaseController::setVelocityFactor(double factor) 
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return;
    }
    
    if (factor < 0.0 || factor > 1.0) 
    {
        RCLCPP_ERROR(logger_, "速度因子必须在0.0-1.0之间");
        return;
    }
    
    move_group_->setMaxVelocityScalingFactor(factor);
    RCLCPP_INFO(logger_, "设置速度因子: %.2f", factor);
}

/**
 * @brief 设置加速度因子
 * @param factor 加速度因子(0.0-1.0)
 */
void CR7BaseController::setAccelerationFactor(double factor) 
{
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return;
    }
    
    if (factor < 0.0 || factor > 1.0) 
    {
        RCLCPP_ERROR(logger_, "加速度因子必须在0.0-1.0之间");
        return;
    }
    
    move_group_->setMaxAccelerationScalingFactor(factor);
    RCLCPP_INFO(logger_, "设置加速度因子: %.2f", factor);
}

/**
 * @brief 结果转字符串（公开方法）
 * @param result 规划结果
 * @return std::string 结果描述
 */
std::string CR7BaseController::resultToString(Result result) const {
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

/**
 * @brief 转换位姿坐标
 * @param input_pose 输入位姿
 * @param output_pose 输出位姿
 * @param target_frame 目标坐标系
 * @param timeout_sec 转换超时时间(秒)
 * @return bool 转换成功返回true
 */
bool CR7BaseController::transformPose(const geometry_msgs::msg::PoseStamped& input_pose,
                                    geometry_msgs::msg::PoseStamped& output_pose,
                                    const std::string& target_frame,
                                    double timeout_sec) 
{
    try 
    {
        // 等待坐标变换可用
        if (!tf_buffer_->canTransform(target_frame, input_pose.header.frame_id,
                                        tf2::TimePointZero,
                                        tf2::durationFromSec(timeout_sec))) 
        {
            RCLCPP_ERROR(logger_, "无法获取从 %s 到 %s 的坐标变换",
                        input_pose.header.frame_id.c_str(), target_frame.c_str());
            return false;
        }
        
        // 执行坐标变换
        output_pose = tf_buffer_->transform(input_pose, target_frame);
        return true;
        
    } 
    catch (tf2::TransformException& ex) 
    {
        RCLCPP_ERROR(logger_, "坐标变换失败: %s", ex.what());
        return false;
    }
}

/**
 * @brief 内部实现移动到位姿
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return Result 规划结果
 */    
CR7BaseController::Result CR7BaseController::moveToPoseImpl(
                                            const geometry_msgs::msg::Pose& target_pose,
                                            const std::string& waypoint_name) 
{
    
    if (!initialized_) 
    {
        RCLCPP_ERROR(logger_, "控制器未初始化");
        return Result::ROBOT_NOT_READY;
    }
    
    // 验证输入
    std::string error_msg;
    if (!validatePose(target_pose, &error_msg)) 
    {
        RCLCPP_ERROR(logger_, "无效的位姿: %s", error_msg.c_str());
        return Result::INVALID_INPUT;
    }
    
    // 打印目标信息
    if (!waypoint_name.empty()) 
    {
        RCLCPP_INFO(logger_, "=====================================");
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
        
        if (!success) 
        {
            RCLCPP_ERROR(logger_, "规划失败 (耗时 %.3f 秒)", planning_time);
            return Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "✓ 规划成功 (耗时 %.3f 秒)", planning_time);
        RCLCPP_INFO(logger_, "轨迹点数: %zu", plan.trajectory_.joint_trajectory.points.size());
        
        // 检查轨迹是否有效
        if (plan.trajectory_.joint_trajectory.points.empty()) 
        {
            RCLCPP_ERROR(logger_, "轨迹为空");
            return Result::PLANNING_FAILED;
        }
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行...");
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) 
        {
            RCLCPP_INFO(logger_, "✓ 执行成功 (耗时 %.3f 秒)", execution_time);
            if (!waypoint_name.empty()) {
                RCLCPP_INFO(logger_, "✓ 到达路点: %s", waypoint_name.c_str());
            }
            return Result::SUCCESS;
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "执行失败 (错误码: %d)", result.val);
            RCLCPP_ERROR(logger_, "耗时: %.3f 秒", execution_time);
            return Result::EXECUTION_FAILED;
        }
        
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "运动控制异常: %s", e.what());
        return Result::EXECUTION_FAILED;
    }
}



/**
 * @brief 验证位姿有效性
 */
bool CR7BaseController::validatePose(const geometry_msgs::msg::Pose& pose, 
                                    std::string* error_msg) const 
{
    // 检查位置是否有效
    if (std::isnan(pose.position.x) || std::isinf(pose.position.x) ||
        std::isnan(pose.position.y) || std::isinf(pose.position.y) ||
        std::isnan(pose.position.z) || std::isinf(pose.position.z)) 
    {
        if (error_msg) 
        {
            *error_msg = "位置包含无效值";
        }
        return false;
    }
    
    // 检查四元数是否有效
    if (std::isnan(pose.orientation.x) || std::isinf(pose.orientation.x) ||
        std::isnan(pose.orientation.y) || std::isinf(pose.orientation.y) ||
        std::isnan(pose.orientation.z) || std::isinf(pose.orientation.z) ||
        std::isnan(pose.orientation.w) || std::isinf(pose.orientation.w)) 
    {
        if (error_msg) 
        {
            *error_msg = "四元数包含无效值";
        }
        return false;
    }
    
    // 检查四元数是否为单位四元数
    double norm = pose.orientation.x*pose.orientation.x + 
                  pose.orientation.y*pose.orientation.y + 
                  pose.orientation.z*pose.orientation.z + 
                  pose.orientation.w*pose.orientation.w;
    
    if (std::abs(norm - 1.0) > 0.1) 
    {
        if (error_msg) 
        {
            *error_msg = "四元数不是单位四元数";
        }
        return false;
    }
    
    return true;
}

}  // namespace cr7_controller
