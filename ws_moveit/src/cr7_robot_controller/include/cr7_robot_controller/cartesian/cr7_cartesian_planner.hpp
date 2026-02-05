/**
 * @file cr7_cartesian_planner.hpp
 * @brief 笛卡尔路径规划模块头文件
 * 
 * 这个文件定义了CR7机器人的笛卡尔路径规划功能，包括：
 * 1. 笛卡尔路径规划参数配置
 * 2. 笛卡尔路径规划和执行
 * 3. 路径预处理和优化
 * 4. 轨迹分析和验证
 */

#ifndef CR7_CARTESIAN_PLANNER_HPP_
#define CR7_CARTESIAN_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

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
 * 
 * 这个类提供：
 * 1. 笛卡尔路径规划和执行
 * 2. 路径预处理和优化
 * 3. 轨迹分析和验证
 * 4. 中间点插入和路径可行性检查
 */
class CR7CartesianPlanner : public CR7BaseController {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param move_group MoveGroup接口指针
     */
    CR7CartesianPlanner(
        rclcpp::Node::SharedPtr node,
        const std::string& planning_group,
        rclcpp::Logger logger
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
        const CartesianPathConfig& config
    );
    
    /**
     * @brief 执行优化的笛卡尔路径（推荐使用）
     * @param waypoints 路径点序列
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executeOptimizedCartesianPath(
        const std::vector<Waypoint>& waypoints
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
     * @brief 计算两点间距离
     */
    double calculateDistance(const geometry_msgs::msg::Pose& pose1,
                           const geometry_msgs::msg::Pose& pose2) const;
    
    /**
     * @brief 插入中间点
     */
    bool insertIntermediatePoints(const geometry_msgs::msg::Pose& start,
                                const geometry_msgs::msg::Pose& end,
                                std::vector<geometry_msgs::msg::Pose>& waypoints,
                                double max_step);
    
    /**
     * @brief 球面线性插值四元数
     */
    bool slerpQuaternion(const geometry_msgs::msg::Quaternion& q1,
                        const geometry_msgs::msg::Quaternion& q2,
                        double t, geometry_msgs::msg::Quaternion& result) const;
    
    /**
     * @brief 尝试简化路径
     */
    double trySimplifiedPath(const std::vector<geometry_msgs::msg::Pose>& original_waypoints,
                           moveit_msgs::msg::RobotTrajectory& trajectory,
                           const CartesianPathConfig& config);
    
    /**
     * @brief 优化轨迹（轻微优化）
     */
    bool optimizeTrajectoryLight(moveit_msgs::msg::RobotTrajectory& trajectory);
    
    /**
     * @brief 检查点是否显著（不应该被删除）
     */
    bool isPointSignificant(const trajectory_msgs::msg::JointTrajectoryPoint& prev,
                          const trajectory_msgs::msg::JointTrajectoryPoint& current,
                          const trajectory_msgs::msg::JointTrajectoryPoint& next) const;
    
    /**
     * @brief 验证轨迹
     */
    bool validateTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    
    /**
     * @brief 保存轨迹分析
     */
    void saveTrajectoryAnalysis(const moveit_msgs::msg::RobotTrajectory& trajectory,
                              const std::string& filename_prefix);
    
    /**
     * @brief 保存路径点分析
     */
    void saveWaypointAnalysis(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                            const std::string& filename_prefix);
    
    /**
     * @brief 打印轨迹信息
     */
    void printTrajectoryInfo(const moveit_msgs::msg::RobotTrajectory& trajectory);
    
    /**
     * @brief 获取当前机器人末端执行器位姿
     */
    geometry_msgs::msg::Pose getCurrentPose();
    
    // 成员变量
    CartesianPathConfig config_;                              ///< 笛卡尔路径配置
};

} // namespace cr7_controller

#endif // CR7_CARTESIAN_PLANNER_HPP_
