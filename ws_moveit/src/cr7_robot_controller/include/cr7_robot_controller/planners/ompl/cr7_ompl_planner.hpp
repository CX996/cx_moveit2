/**
 * @file cr7_ompl_planner.hpp
 * @brief OMPL规划器模块头文件
 * 
 * 这个文件定义了CR7机器人的OMPL规划器功能，包括：
 * 1. OMPL规划器配置
 * 2. 基于OMPL的路径规划
 * 3. 规划结果执行
 * 4. 规划参数管理
 */

#ifndef CR7_OMPL_PLANNER_HPP_
#define CR7_OMPL_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

// 添加TF2头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"

namespace cr7_controller {

/**
 * @brief OMPL规划配置结构
 */
struct OMPLConfig {
    std::string planner_id;          ///< 规划器ID
    double planning_time;            ///< 规划时间(秒)
    int num_planning_attempts;       ///< 规划尝试次数
    double velocity_scale;           ///< 速度缩放因子
    double acceleration_scale;       ///< 加速度缩放因子
    double goal_position_tolerance;  ///< 目标位置容差
    double goal_orientation_tolerance; ///< 目标姿态容差

    OMPLConfig() 
        : planner_id("RRTstarkConfigDefault"),
          planning_time(10.0),
          num_planning_attempts(100),
          velocity_scale(0.3),
          acceleration_scale(0.3),
          goal_position_tolerance(0.001),
          goal_orientation_tolerance(0.01),
    {
    }
};

/**
 * @class CR7OMPLPlanner
 * @brief OMPL规划器类
 * 
 * 这个类提供：
 * 1. 基于OMPL的路径规划
 * 2. 规划结果执行
 * 3. 规划参数配置
 * 4. 规划结果分析
 */
class CR7OMPLPlanner {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param move_group MoveGroup接口指针
     * @param logger 日志记录器
     */
    CR7OMPLPlanner(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group
    );
    
    /**
     * @brief 析构函数
     */
    ~CR7OMPLPlanner() = default;
    
    /**
     * @brief 移动到单个位姿
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPose(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 移动到单个位姿，支持PoseStamped
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPose(
        const geometry_msgs::msg::PoseStamped& target_pose,
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 执行多路点序列
     * @param waypoints 路点向量
     * @param delay_seconds 路点间延迟(秒)
     * @return std::vector<CR7BaseController::Result> 每个路点的结果
     */
    std::vector<CR7BaseController::Result> executeWaypoints(
        const std::vector<Waypoint>& waypoints,
        double delay_seconds = 2.0
    );
    
    /**
     * @brief 设置规划时间
     * @param seconds 规划时间(秒)
     */
    void setPlanningTime(double seconds);
    
    /**
     * @brief 设置速度因子
     * @param factor 速度因子(0.0-1.0)
     */
    void setVelocityFactor(double factor);
    
    /**
     * @brief 设置加速度因子
     * @param factor 加速度因子(0.0-1.0)
     */
    void setAccelerationFactor(double factor);
    
    /**
     * @brief 获取OMPL规划配置
     * @return OMPLConfig& 配置引用
     */
    OMPLConfig& getConfig() 
    {
        return config_;
    }
    
    /**
     * @brief 设置位置约束（盒子约束）
     * @param link_name 要约束的连杆名称
     * @param min_x 最小x坐标
     * @param max_x 最大x坐标
     * @param min_y 最小y坐标
     * @param max_y 最大y坐标
     * @param min_z 最小z坐标
     * @param max_z 最大z坐标
     * @param frame_id 参考坐标系
     */
    void setPositionConstraintBox(
        const std::string& link_name,
        double min_x, double max_x,
        double min_y, double max_y,
        double min_z, double max_z,
        const std::string& frame_id = "base_link"
    );
    
    /**
     * @brief 设置位置约束（平面约束）
     * @param link_name 要约束的连杆名称
     * @param plane_normal 平面法线
     * @param distance 平面距离原点的距离
     * @param frame_id 参考坐标系
     */
    void setPositionConstraintPlane(
        const std::string& link_name,
        const geometry_msgs::msg::Vector3& plane_normal,
        double distance,
        const std::string& frame_id = "base_link"
    );
    
    /**
     * @brief 设置位置约束（直线约束）
     * @param link_name 要约束的连杆名称
     * @param line_start 直线起点
     * @param line_end 直线终点
     * @param frame_id 参考坐标系
     */
    void setPositionConstraintLine(
        const std::string& link_name,
        const geometry_msgs::msg::Point& line_start,
        const geometry_msgs::msg::Point& line_end,
        const std::string& frame_id = "base_link"
    );
    
    /**
     * @brief 设置姿态约束
     * @param link_name 要约束的连杆名称
     * @param orientation 目标姿态
     * @param tolerance_x x轴方向容差
     * @param tolerance_y y轴方向容差
     * @param tolerance_z z轴方向容差
     * @param frame_id 参考坐标系
     */
    void setOrientationConstraint(
        const std::string& link_name,
        const geometry_msgs::msg::Quaternion& orientation,
        double tolerance_x = 0.01,
        double tolerance_y = 0.01,
        double tolerance_z = 0.01,
        const std::string& frame_id = "base_link"
    );
    
    /**
     * @brief 清除所有约束
     */
    void clearConstraints();
    
    /**
     * @brief 执行带约束的规划
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithConstraints(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& waypoint_name = ""
    );
    
private:
    /**
     * @brief 内部实现移动到位姿
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */                                     
    CR7BaseController::Result moveToPoseImpl(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& waypoint_name
    );
    
    // 成员变量
    rclcpp::Node::SharedPtr node_;                          ///< ROS节点
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; ///< MoveIt接口
    rclcpp::Logger logger_;                                 ///< 日志记录器
    OMPLConfig config_;                                     ///< OMPL规划配置
    
    // 添加坐标变换监听器
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; ///< TF2缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< TF2监听器
};

}  // namespace cr7_controller

#endif  // CR7_OMPL_PLANNER_HPP_
