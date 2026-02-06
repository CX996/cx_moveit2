/**
 * @file cr7_cartesian_planner.hpp
 * @brief 笛卡尔路径规划模块头文件
 * 
 * 这个文件定义了CR7机器人的笛卡尔路径规划功能
 */

#ifndef CR7_CARTESIAN_PLANNER_HPP_
#define CR7_CARTESIAN_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

// 添加TF2头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"

namespace cr7_controller {

/**
 * @brief 笛卡尔路径规划参数结构
 */
struct CartesianPathConfig {
    double max_step;           ///< 最大步长 (米)
    double jump_threshold;     ///< 跳跃阈值 (0.0表示禁用)
    double eef_step;           ///< 末端执行器步长
    bool avoid_collisions;     ///< 是否避障
    double max_velocity_factor; ///< 最大速度因子
    
    /**
     * @brief 构造函数，设置默认值
     */
    CartesianPathConfig() 
        : max_step(0.001), jump_threshold(2.0), eef_step(0.001), 
          avoid_collisions(true), max_velocity_factor(0.3)
    {
    }
};


/**
 * @class CR7CartesianPlanner
 * @brief 笛卡尔路径规划器类
 */
class CR7CartesianPlanner {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param move_group MoveGroup接口指针
     * @param logger 日志记录器
     */
    CR7CartesianPlanner(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group
    );
    
    /**
     * @brief 析构函数
     */
    ~CR7CartesianPlanner() = default;
    
    /**
     * @brief 笛卡尔路径规划
     * @param waypoints 路径点序列
     * @param config 笛卡尔路径配置
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executeCartesianPath(
        const std::vector<Waypoint>& waypoints,
        const CartesianPathConfig& config = CartesianPathConfig()
    );
    
    /**
     * @brief 获取笛卡尔路径配置
     * @return CartesianPathConfig& 配置引用
     */
    CartesianPathConfig& getConfig() 
    {
        return config_;
    }
    
private:
    /**
     * @brief 预处理路径点
     */
    bool preprocessWaypoints(const std::vector<Waypoint>& input_waypoints,
                           std::vector<geometry_msgs::msg::Pose>& output_poses,
                           const CartesianPathConfig& config);
    
    /**
     * @brief 检查路径可行性
     */
    bool checkPathFeasibility(const std::vector<geometry_msgs::msg::Pose>& waypoints);
    
    /**
     * @brief 验证轨迹
     */
    bool validateTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    
    /**
     * @brief 获取当前机器人末端执行器位姿
     */
    geometry_msgs::msg::Pose getCurrentPose();
    
    // 成员变量
    rclcpp::Node::SharedPtr node_;                          ///< ROS节点
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; ///< MoveIt接口
    rclcpp::Logger logger_;                                 ///< 日志记录器
    CartesianPathConfig config_;                              ///< 笛卡尔路径配置
    
    // 添加坐标变换监听器
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; ///< TF2缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< TF2监听器
};

} // namespace cr7_controller

#endif // CR7_CARTESIAN_PLANNER_HPP_
