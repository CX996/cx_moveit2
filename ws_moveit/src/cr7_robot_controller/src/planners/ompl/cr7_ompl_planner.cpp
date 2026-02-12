/**
 * @file cr7_ompl_planner.cpp
 * @brief OMPL规划器模块实现文件
 * 
 * 实现CR7OMPLPlanner类的所有方法
 */

#include <cmath>
#include <chrono>
#include <sstream>
#include <iomanip>

#include "cr7_robot_controller/planners/ompl/cr7_ompl_planner.hpp"
#include "cr7_robot_controller/utils/pose_utils.hpp"
#include "cr7_robot_controller/utils/trajectory_analyzer.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>

using namespace std::chrono_literals;

namespace cr7_controller {

// ============================================================================
// CR7OMPLPlanner 方法实现
// ============================================================================

/**
 * @brief 构造函数
 * @param node ROS节点指针
 * @param move_group MoveGroup接口指针
 * @param logger 日志记录器
 */
CR7OMPLPlanner::CR7OMPLPlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group
)   : node_(node)
    , move_group_(move_group)
    , logger_(node->get_logger())
{
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 初始化MoveItVisualTools
    std::string base_frame = move_group_->getPlanningFrame();
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        node, base_frame, rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_->getRobotModel());
    visual_tools_->loadRemoteControl();

    RCLCPP_INFO(logger_, "创建OMPL规划器");
    RCLCPP_INFO(logger_, "默认规划器: %s", config_.planner_id.c_str());
    RCLCPP_INFO(logger_, "初始化可视化工具，参考坐标系: %s", base_frame.c_str());

    // 设置默认规划参数
    move_group_->setPlannerId(config_.planner_id);
    move_group_->setPlanningTime(config_.planning_time);
    move_group_->setNumPlanningAttempts(config_.num_planning_attempts);
    move_group_->setMaxVelocityScalingFactor(config_.velocity_scale);
    move_group_->setMaxAccelerationScalingFactor(config_.acceleration_scale);
    move_group_->setGoalPositionTolerance(config_.goal_position_tolerance);
    move_group_->setGoalOrientationTolerance(config_.goal_orientation_tolerance);
    
    // 添加约束规划的特殊设置
    move_group_->setWorkspace(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);  // 设置更大的工作空间
    move_group_->setStartStateToCurrentState();  // 使用当前状态作为起始状态

    // 设置约束规划参数
    // 注意：这些参数也需要在ompl_planning.yaml文件中设置
    // enforce_constrained_state_space: true
    // projection_evaluator: "joints(joint_1,joint_2)"
    
    RCLCPP_INFO(logger_, "OMPL规划器初始化完成");
    RCLCPP_INFO(logger_, "约束规划参数设置提示:");
    RCLCPP_INFO(logger_, "请在ompl_planning.yaml文件中添加以下参数:");
    RCLCPP_INFO(logger_, "cr7_group:");
    RCLCPP_INFO(logger_, "  enforce_constrained_state_space: true");
    RCLCPP_INFO(logger_, "  projection_evaluator: \"joints(joint_1,joint_2)\"");
    RCLCPP_INFO(logger_, "可视化工具已就绪，可在RViz中查看约束区域和规划轨迹");
}

/**
 * @brief 移动到单个位姿
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPose(const geometry_msgs::msg::Pose& target_pose,
                                                   const std::string& waypoint_name) 
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = move_group_->getPlanningFrame(); // 默认规划坐标系
    pose_stamped.pose = target_pose;
    return moveToPose(pose_stamped, waypoint_name);
}

/**
 * @brief 移动到单个位姿，支持PoseStamped
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return CR7BaseController::Result 规划结果
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPose(const geometry_msgs::msg::PoseStamped& target_pose,
                                                   const std::string& waypoint_name) 
{
    // 获取规划坐标系（通常是机器人的基坐标系）
    std::string planning_frame = move_group_->getPlanningFrame();
    
    geometry_msgs::msg::PoseStamped transformed_pose;
    
    // 如果目标坐标系与规划坐标系不同，进行坐标变换
    if (target_pose.header.frame_id != planning_frame) {
        RCLCPP_INFO(logger_, "变换坐标系: %s -> %s", 
                    target_pose.header.frame_id.c_str(), planning_frame.c_str());
        
        if (!utils::PoseUtils::transformPose(target_pose, transformed_pose, planning_frame, tf_buffer_, 2.0, logger_)) {
            RCLCPP_ERROR(logger_, "坐标变换失败");
            return CR7BaseController::Result::INVALID_INPUT;
        }
    } else {
        transformed_pose = target_pose;
    }
    
    return moveToPoseImpl(transformed_pose.pose, waypoint_name);
}

/**
 * @brief 执行多路点序列
 * @param waypoints 路点向量
 * @param delay_seconds 路点间延迟(秒)
 * @return std::vector<CR7BaseController::Result> 每个路点的结果
 */
std::vector<CR7BaseController::Result> CR7OMPLPlanner::executeWaypoints(
    const std::vector<Waypoint>& waypoints,
    double delay_seconds) 
{
    std::vector<CR7BaseController::Result> results;
    
    RCLCPP_INFO(logger_, "开始执行 %zu 个路点，路点间延迟: %.1f 秒", waypoints.size(), delay_seconds);
    
    int success_count = 0;
    
    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        const auto& wp = waypoints[i];
        
        RCLCPP_INFO(logger_, "执行路点 %zu/%zu: %s", i + 1, waypoints.size(), wp.name.c_str());
        
        // 执行路点
        auto result = moveToPose(wp.toPoseStamped(), wp.name);
        results.push_back(result);
        
        if (result == CR7BaseController::Result::SUCCESS) 
        {
            success_count++;
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "路点 %s 失败: %s", 
                        wp.name.c_str(), utils::PoseUtils::resultToString(result).c_str());
        }
        
        // 路点间延迟（最后一个路点后不延迟）
        if (i < waypoints.size() - 1 && delay_seconds > 0) 
        {
            RCLCPP_INFO(logger_, "等待 %.1f 秒...", delay_seconds);
            // 方法1：使用 duration_cast
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(delay_seconds)));
        }
    }
    
    RCLCPP_INFO(logger_, "路点序列执行完成，成功: %d/%zu", success_count, waypoints.size());
    
    return results;
}

/**
 * @brief 设置规划时间
 * @param seconds 规划时间(秒)
 */
void CR7OMPLPlanner::setPlanningTime(double seconds) 
{
    config_.planning_time = seconds;
    move_group_->setPlanningTime(seconds);
    RCLCPP_INFO(logger_, "设置规划时间: %.1f秒", seconds);
}

/**
 * @brief 设置速度因子
 * @param factor 速度因子(0.0-1.0)
 */
void CR7OMPLPlanner::setVelocityFactor(double factor) 
{
    if (factor < 0.0 || factor > 1.0) 
    {
        RCLCPP_ERROR(logger_, "速度因子必须在0.0-1.0之间");
        return;
    }
    
    config_.velocity_scale = factor;
    move_group_->setMaxVelocityScalingFactor(factor);
    RCLCPP_INFO(logger_, "设置速度因子: %.2f", factor);
}

/**
 * @brief 设置加速度因子
 * @param factor 加速度因子(0.0-1.0)
 */
void CR7OMPLPlanner::setAccelerationFactor(double factor) 
{
    if (factor < 0.0 || factor > 1.0) 
    {
        RCLCPP_ERROR(logger_, "加速度因子必须在0.0-1.0之间");
        return;
    }
    
    config_.acceleration_scale = factor;
    move_group_->setMaxAccelerationScalingFactor(factor);
    RCLCPP_INFO(logger_, "设置加速度因子: %.2f", factor);
}

/**
 * @brief 内部实现移动到位姿
 * @param target_pose 目标位姿
 * @param waypoint_name 路点名称（用于日志）
 * @return CR7BaseController::Result 规划结果
 */                                     
CR7BaseController::Result CR7OMPLPlanner::moveToPoseImpl(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& waypoint_name) 
{
    // 验证输入
    std::string error_msg;
    if (!utils::PoseUtils::validatePose(target_pose, &error_msg)) 
    {
        RCLCPP_ERROR(logger_, "无效的位姿: %s", error_msg.c_str());
        return CR7BaseController::Result::INVALID_INPUT;
    }
    
    // 打印目标信息
    if (!waypoint_name.empty()) 
    {
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
        
        if (!success || plan.trajectory_.joint_trajectory.points.empty()) 
        {
            RCLCPP_ERROR(logger_, "规划失败 (耗时 %.3f 秒)", planning_time);
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "规划成功 (耗时 %.3f 秒), 轨迹点数: %zu", planning_time, plan.trajectory_.joint_trajectory.points.size());
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行...");
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) 
        {
            RCLCPP_INFO(logger_, "执行成功 (耗时 %.3f 秒)", execution_time);
            if (!waypoint_name.empty()) 
            {
                RCLCPP_INFO(logger_, "到达路点: %s", waypoint_name.c_str());
            }
            return CR7BaseController::Result::SUCCESS;
        } 
        else 
        {
            RCLCPP_ERROR(logger_, "执行失败 (错误码: %d, 耗时: %.3f 秒)", result.val, execution_time);
            return CR7BaseController::Result::EXECUTION_FAILED;
        }
        
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(logger_, "运动控制异常: %s", e.what());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

/**
 * @brief 设置位置约束（盒子约束）
 */
void CR7OMPLPlanner::setPositionConstraintBox(
    const std::string& link_name,
    double min_x, double max_x,
    double min_y, double max_y,
    double min_z, double max_z,
    const std::string& frame_id
) 
{
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint position_constraint;
    
    position_constraint.header.frame_id = frame_id;
    position_constraint.link_name = link_name;
    position_constraint.weight = 1.0;
    
    // 创建盒子约束
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = max_x - min_x;
    box.dimensions[1] = max_y - min_y;
    box.dimensions[2] = max_z - min_z;
    
    // 盒子中心位置
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = (min_x + max_x) / 2.0;
    box_pose.position.y = (min_y + max_y) / 2.0;
    box_pose.position.z = (min_z + max_z) / 2.0;
    box_pose.orientation.w = 1.0;
    
    position_constraint.constraint_region.primitives.push_back(box);
    position_constraint.constraint_region.primitive_poses.push_back(box_pose);
    
    constraints.position_constraints.push_back(position_constraint);
    constraints.name = "use_equality_constraints";
    move_group_->setPathConstraints(constraints);
    
    // 可视化盒子约束
    if (visual_tools_) {
        Eigen::Vector3d box_point_1(min_x, min_y, min_z);
        Eigen::Vector3d box_point_2(max_x, max_y, max_z);
        visual_tools_->publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
        visual_tools_->trigger();
    }
    
    // 为盒子约束选择合适的规划器
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    
    RCLCPP_INFO(logger_, "设置位置约束（盒子约束）: 连杆=%s, 坐标系=%s", link_name.c_str(), frame_id.c_str());
    RCLCPP_INFO(logger_, "盒子尺寸: x=[%.3f,%.3f], y=[%.3f,%.3f], z=[%.3f,%.3f]", min_x, max_x, min_y, max_y, min_z, max_z);
    RCLCPP_INFO(logger_, "为盒子约束选择规划器: RRTConnectkConfigDefault");
}

/**
     * @brief 设置位置约束（平面约束）
     */
    void CR7OMPLPlanner::setPositionConstraintPlane(
        const std::string& link_name,
        const geometry_msgs::msg::Vector3& plane_normal,
        double distance,
        const std::string& frame_id
    ) 
    {
        moveit_msgs::msg::Constraints constraints;
        moveit_msgs::msg::PositionConstraint position_constraint;
        
        position_constraint.header.frame_id = frame_id;
        position_constraint.link_name = link_name;
        position_constraint.weight = 1.0;
        
        // 创建一个平面约束
        // 注意：根据官网教程，我们需要使用一个长方体来模拟平面
        // 长方体的厚度很小，以模拟平面
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions.resize(3);
        box.dimensions[0] = 1.0; // 长度
        box.dimensions[1] = 1.0; // 宽度
        box.dimensions[2] = 0.001; // 厚度（适度增加，提高规划成功率）
        
        // 计算平面的位置
        geometry_msgs::msg::Pose box_pose;
        // 平面位置在距离原点distance处
        // 方向与平面法线一致
        
        // 计算从z轴到平面法线的旋转
        geometry_msgs::msg::Vector3 z_axis;
        z_axis.x = 0.0;
        z_axis.y = 0.0;
        z_axis.z = 1.0;
        
        // 计算旋转轴
        geometry_msgs::msg::Vector3 rotation_axis;
        rotation_axis.x = z_axis.y * plane_normal.z - z_axis.z * plane_normal.y;
        rotation_axis.y = z_axis.z * plane_normal.x - z_axis.x * plane_normal.z;
        rotation_axis.z = z_axis.x * plane_normal.y - z_axis.y * plane_normal.x;
        
        // 计算旋转角度
        double dot_product = z_axis.x * plane_normal.x + z_axis.y * plane_normal.y + z_axis.z * plane_normal.z;
        double angle = std::acos(dot_product);
        
        // 计算四元数
        double sin_half_angle = std::sin(angle / 2.0);
        double rotation_axis_length = std::sqrt(
            rotation_axis.x * rotation_axis.x +
            rotation_axis.y * rotation_axis.y +
            rotation_axis.z * rotation_axis.z
        );
        
        if (rotation_axis_length > 0) 
        {
            box_pose.orientation.x = rotation_axis.x / rotation_axis_length * sin_half_angle;
            box_pose.orientation.y = rotation_axis.y / rotation_axis_length * sin_half_angle;
            box_pose.orientation.z = rotation_axis.z / rotation_axis_length * sin_half_angle;
            box_pose.orientation.w = std::cos(angle / 2.0);
        } 
        else 
        {
            box_pose.orientation.w = 1.0;
        }
        
        // 设置平面位置
        box_pose.position.x = plane_normal.x * distance;
        box_pose.position.y = plane_normal.y * distance;
        box_pose.position.z = plane_normal.z * distance;
        
        // 添加长方体到约束区域
        position_constraint.constraint_region.primitives.push_back(box);
        position_constraint.constraint_region.primitive_poses.push_back(box_pose);
        
        constraints.position_constraints.push_back(position_constraint);
        constraints.name = "use_equality_constraints";
        move_group_->setPathConstraints(constraints);
        
        // 为平面约束选择合适的规划器
        move_group_->setPlannerId("RRTConnectkConfigDefault");
        
        // 可视化平面约束
        if (visual_tools_) 
        {
            // 计算平面上的点用于可视化
            Eigen::Vector3d normal(plane_normal.x, plane_normal.y, plane_normal.z);
            visual_tools_->publishNormalAndDistancePlane(normal, distance, rviz_visual_tools::TRANSLUCENT_DARK);
            visual_tools_->trigger();
        }
        
        RCLCPP_INFO(logger_, "设置位置约束（平面约束）: 连杆=%s, 坐标系=%s", link_name.c_str(), frame_id.c_str());
        RCLCPP_INFO(logger_, "平面法线: [%.3f,%.3f,%.3f], 距离: %.3f", plane_normal.x, plane_normal.y, plane_normal.z, distance);
        RCLCPP_INFO(logger_, "为平面约束选择规划器: RRTConnectkConfigDefault");
    }

/**
 * @brief 设置位置约束（直线约束）- 修正版
 */
void CR7OMPLPlanner::setPositionConstraintLine(
    const std::string& link_name,
    const geometry_msgs::msg::Point& line_start,
    const geometry_msgs::msg::Point& line_end,
    const std::string& frame_id
) 
{
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint position_constraint;
    
    position_constraint.header.frame_id = frame_id;
    position_constraint.link_name = link_name;
    position_constraint.weight = 1.0;
    
    // 计算直线方向和长度
    double dx = line_end.x - line_start.x;
    double dy = line_end.y - line_start.y;
    double dz = line_end.z - line_start.z;
    
    double line_length = std::sqrt(dx*dx + dy*dy + dz*dz);
    
    if (line_length < 1e-6) {
        RCLCPP_ERROR(logger_, "直线长度太短: %.6f", line_length);
        return;
    }
    
    // 归一化方向向量
    dx /= line_length;
    dy /= line_length;
    dz /= line_length;
    
    // 创建表示直线的长方体约束
    // 关键修改：非主轴方向尺寸要小于0.001
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = line_length;  // 主轴方向：直线长度
    box.dimensions[1] = 0.0005;       // 官方文档要求
    box.dimensions[2] = 0.0005;       // 官方文档要求
    
    // 长方体中心点
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = (line_start.x + line_end.x) / 2.0;
    box_pose.position.y = (line_start.y + line_end.y) / 2.0;
    box_pose.position.z = (line_start.z + line_end.z) / 2.0;
    
    // 简化姿态计算
    // 将长方体主轴（X轴）对齐到直线方向
    // 使用tf2的quaternionFromUnitVectors或手动计算
    tf2::Vector3 start_vec(line_start.x, line_start.y, line_start.z);
    tf2::Vector3 end_vec(line_end.x, line_end.y, line_end.z);
    tf2::Vector3 line_vec = end_vec - start_vec;
    line_vec.normalize();
    
    // 长方体默认X轴是主轴，我们要将其旋转到直线方向
    tf2::Vector3 x_axis(1, 0, 0);  // 长方体默认主轴方向
    tf2::Quaternion rotation = tf2::shortestArcQuat(x_axis, line_vec);
    rotation.normalize();
    
    // 转换回geometry_msgs
    box_pose.orientation.x = rotation.x();
    box_pose.orientation.y = rotation.y();
    box_pose.orientation.z = rotation.z();
    box_pose.orientation.w = rotation.w();
    
    // 添加约束区域
    position_constraint.constraint_region.primitives.push_back(box);
    position_constraint.constraint_region.primitive_poses.push_back(box_pose);
    constraints.position_constraints.push_back(position_constraint);
    
    // 关键：设置等式约束标志
    constraints.name = "use_equality_constraints";
    
    // 设置路径约束
    move_group_->setPathConstraints(constraints);
    
    // 为直线约束选择合适的规划器
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    
    // 可视化直线约束
    if (visual_tools_) 
    {
        visual_tools_->publishLine(line_start, line_end, rviz_visual_tools::TRANSLUCENT_DARK);
        visual_tools_->trigger();
    }
    
    RCLCPP_INFO(logger_, "设置直线约束: 连杆=%s, 坐标系=%s", link_name.c_str(), frame_id.c_str());
    RCLCPP_INFO(logger_, "直线长度: %.3f, 方向: [%.3f,%.3f,%.3f]", 
                line_length, dx, dy, dz);
    RCLCPP_INFO(logger_, "约束尺寸: [%.6f,%.6f,%.6f]", 
                box.dimensions[0], box.dimensions[1], box.dimensions[2]);
    RCLCPP_INFO(logger_, "为直线约束选择规划器: RRTConnectkConfigDefault");
}

/**
 * @brief 设置姿态约束
 */
void CR7OMPLPlanner::setOrientationConstraint(
    const std::string& link_name,
    const geometry_msgs::msg::Quaternion& orientation,
    double tolerance_x,
    double tolerance_y,
    double tolerance_z,
    const std::string& frame_id
) 
{
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::OrientationConstraint orient_constraint;
    
    orient_constraint.header.frame_id = frame_id;
    orient_constraint.link_name = link_name;
    orient_constraint.orientation = orientation;

    // 适度增加姿态约束容忍度，提高规划成功率
    orient_constraint.absolute_x_axis_tolerance = std::max(tolerance_x, 0.05);
    orient_constraint.absolute_y_axis_tolerance = std::max(tolerance_y, 0.05);
    orient_constraint.absolute_z_axis_tolerance = std::max(tolerance_z, 0.05);
    orient_constraint.weight = 1.0;
    
    constraints.orientation_constraints.push_back(orient_constraint);
    move_group_->setPathConstraints(constraints);
    
    // 为姿态约束选择合适的规划器
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    
    // 可视化姿态约束
    if (visual_tools_) 
    {
        // 创建一个临时位姿来显示姿态约束的坐标轴
        geometry_msgs::msg::Pose pose;
        pose.orientation = orientation;
        // 使用当前位置作为显示位置
        auto current_pose = move_group_->getCurrentPose();
        pose.position = current_pose.pose.position;
        visual_tools_->publishAxis(pose, 0.1);
        visual_tools_->trigger();
    }
    
    RCLCPP_INFO(logger_, "设置姿态约束: 连杆=%s, 坐标系=%s", link_name.c_str(), frame_id.c_str());
    RCLCPP_INFO(logger_, "目标姿态: [%.3f,%.3f,%.3f,%.3f]", 
                orientation.x, orientation.y, orientation.z, orientation.w);
    RCLCPP_INFO(logger_, "容差: x=%.3f, y=%.3f, z=%.3f", tolerance_x, tolerance_y, tolerance_z);
    RCLCPP_INFO(logger_, "为姿态约束选择规划器: RRTConnectkConfigDefault");
}

/**
 * @brief 清除所有约束
 */
void CR7OMPLPlanner::clearConstraints() 
{
    move_group_->clearPathConstraints();
    
    // 清除可视化标记
    if (visual_tools_) {
        visual_tools_->deleteAllMarkers();
        visual_tools_->trigger();
    }
    
    RCLCPP_INFO(logger_, "清除所有约束和可视化标记");
}

/**
 * @brief 执行带约束的规划
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPoseWithConstraints(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& waypoint_name
) {
    // 验证输入
    std::string error_msg;
    if (!utils::PoseUtils::validatePose(target_pose, &error_msg)) {
        RCLCPP_ERROR(logger_, "无效的位姿: %s", error_msg.c_str());
        return CR7BaseController::Result::INVALID_INPUT;
    }
    
    // 打印目标信息
    if (!waypoint_name.empty()) {
        RCLCPP_INFO(logger_, "移动到路点（带约束）: %s", waypoint_name.c_str());
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
        RCLCPP_INFO(logger_, "开始带约束的规划...");
        
        // 记录规划器信息
        RCLCPP_INFO(logger_, "当前规划器: %s", move_group_->getPlannerId().c_str());
        RCLCPP_INFO(logger_, "规划时间限制: %.1f 秒", move_group_->getPlanningTime());
        RCLCPP_INFO(logger_, "规划尝试次数: %d", move_group_->getNumPlanningAttempts());
        
        auto start_time = node_->now();
        bool success = static_cast<bool>(move_group_->plan(plan));
        double planning_time = (node_->now() - start_time).seconds();
        
        if (!success || plan.trajectory_.joint_trajectory.points.empty()) 
        {   
            RCLCPP_ERROR(logger_, "带约束的规划失败 (耗时 %.3f 秒)", planning_time);
            RCLCPP_ERROR(logger_, "规划失败原因分析:");
            RCLCPP_ERROR(logger_, "1. 约束条件可能过于严格");
            RCLCPP_ERROR(logger_, "2. 目标位姿可能不可达");
            RCLCPP_ERROR(logger_, "3. 规划时间可能不足");
            return CR7BaseController::Result::PLANNING_FAILED;
        }
        
        RCLCPP_INFO(logger_, "带约束的规划成功 (耗时 %.3f 秒), 轨迹点数: %zu", planning_time, plan.trajectory_.joint_trajectory.points.size());
        
        // 分析轨迹长度
        if (!plan.trajectory_.joint_trajectory.points.empty()) {
            size_t num_points = plan.trajectory_.joint_trajectory.points.size();
            double trajectory_duration = plan.trajectory_.joint_trajectory.points.back().time_from_start.sec + 
                                       plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec / 1e9;
            RCLCPP_INFO(logger_, "轨迹持续时间: %.3f 秒, 平均点数: %.1f 点/秒", trajectory_duration, num_points / trajectory_duration);
        }
        
        // 检查轨迹是否为直线（如果是直线约束）
        if (waypoint_name.find("line_constraint") != std::string::npos) 
        {   
            // 获取当前位姿作为起点
            auto current_pose = move_group_->getCurrentPose().pose;
            
            // 检查轨迹是否为直线
            bool is_linear = cr7_controller::utils::TrajectoryAnalyzer::isTrajectoryLinear(
                            plan.trajectory_, current_pose, target_pose, 0.001, move_group_, logger_
            );
            
            if (is_linear) 
            {   
                RCLCPP_INFO(logger_, "轨迹检查: 轨迹是直线");
            } 
            else 
            {   
                RCLCPP_WARN(logger_, "轨迹检查: 轨迹不是直线");
                RCLCPP_WARN(logger_, "可能原因: 约束厚度不足或规划器参数需要调整");
            }
        }
        
        // 执行规划
        RCLCPP_INFO(logger_, "开始执行带约束的运动...");
        start_time = node_->now();
        auto result = move_group_->execute(plan);
        double execution_time = (node_->now() - start_time).seconds();
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS) 
        {   
            RCLCPP_INFO(logger_, "带约束的运动执行成功 (耗时 %.3f 秒)", execution_time);
            if (!waypoint_name.empty()) {
                RCLCPP_INFO(logger_, "到达路点（带约束）: %s", waypoint_name.c_str());
            }
            RCLCPP_INFO(logger_, "规划性能指标:");
            RCLCPP_INFO(logger_, "- 规划时间: %.3f 秒", planning_time);
            RCLCPP_INFO(logger_, "- 执行时间: %.3f 秒", execution_time);
            RCLCPP_INFO(logger_, "- 总时间: %.3f 秒", planning_time + execution_time);
            return CR7BaseController::Result::SUCCESS;
        } 
        else 
        {   
            RCLCPP_ERROR(logger_, "带约束的运动执行失败 (错误码: %d, 耗时: %.3f 秒)", result.val, execution_time);
            RCLCPP_ERROR(logger_, "执行失败原因分析:");
            RCLCPP_ERROR(logger_, "1. 轨迹可能超出关节限制");
            RCLCPP_ERROR(logger_, "2. 执行过程中可能发生碰撞");
            RCLCPP_ERROR(logger_, "3. 控制器可能无法跟随轨迹");
            return CR7BaseController::Result::EXECUTION_FAILED;
        }
        
    } 
    catch (const std::exception& e) 
    {   
        RCLCPP_ERROR(logger_, "带约束的运动控制异常: %s", e.what());
        RCLCPP_ERROR(logger_, "异常类型: %s", typeid(e).name());
        return CR7BaseController::Result::EXECUTION_FAILED;
    }
}

/**
 * @brief 执行带盒子约束的规划
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPoseWithBoxConstraint(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& link_name,
    double min_x, double max_x,
    double min_y, double max_y,
    double min_z, double max_z,
    const std::string& frame_id,
    const std::string& waypoint_name
) {
    // 设置盒子约束
    setPositionConstraintBox(link_name, min_x, max_x, min_y, max_y, min_z, max_z, frame_id);
    
    // 执行规划
    auto result = moveToPoseWithConstraints(target_pose, waypoint_name);
    
    // 清除约束
    clearConstraints();
    
    return result;
}

/**
 * @brief 执行带平面约束的规划
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPoseWithPlaneConstraint(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& link_name,
    const geometry_msgs::msg::Vector3& plane_normal,
    double distance,
    const std::string& frame_id,
    const std::string& waypoint_name
) {
    // 设置平面约束
    setPositionConstraintPlane(link_name, plane_normal, distance, frame_id);
    
    // 执行规划
    auto result = moveToPoseWithConstraints(target_pose, waypoint_name);
    
    // 清除约束
    clearConstraints();
    
    return result;
}

/**
 * @brief 执行带直线约束的规划
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPoseWithLineConstraint(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& link_name,
    const geometry_msgs::msg::Point& line_start,
    const geometry_msgs::msg::Point& line_end,
    const std::string& frame_id,
    const std::string& waypoint_name
) {
    // 清除当前约束
    move_group_->clearPathConstraints(); // 清除约束条件

    // 设置直线约束
    setPositionConstraintLine(link_name, line_start, line_end, frame_id);
    
    // 执行规划
    auto result = moveToPoseWithConstraints(target_pose, waypoint_name);
    
    // 清除约束
    clearConstraints();
    
    // 这里可以添加轨迹直线检查
    // 注意：由于我们已经执行了规划，这里无法直接获取轨迹
    // 实际应用中，可能需要修改moveToPoseWithConstraints函数，返回轨迹信息
    
    return result;
}

/**
 * @brief 执行带姿态约束的规划
 */
CR7BaseController::Result CR7OMPLPlanner::moveToPoseWithOrientationConstraint(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& link_name,
    const geometry_msgs::msg::Quaternion& orientation,
    double tolerance_x,
    double tolerance_y,
    double tolerance_z,
    const std::string& frame_id,
    const std::string& waypoint_name
) {
    // 设置姿态约束
    setOrientationConstraint(link_name, orientation, tolerance_x, tolerance_y, tolerance_z, frame_id);
    
    // 执行规划
    auto result = moveToPoseWithConstraints(target_pose, waypoint_name);
    
    // 清除约束
    clearConstraints();
    
    return result;
}

}  // namespace cr7_controller
