/**
 * @file cr7_pilz_planner.hpp
 * @brief PILZ工业规划器模块头文件
 * 
 * 这个文件定义了CR7机器人的PILZ工业规划器功能，包括：
 * 1. PILZ规划器类型和配置
 * 2. PILZ规划器可用性检查
 * 3. PILZ规划器执行（LIN、PTP、CIRC）
 * 4. PILZ规划结果分析
 */

#ifndef CR7_PILZ_PLANNER_HPP_
#define CR7_PILZ_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"

namespace cr7_controller {

/**
 * @brief PILZ规划器类型枚举
 */
enum class PilzPlanner {
    LIN,    ///< 直线运动
    PTP,    ///< 点对点运动
    CIRC    ///< 圆周运动
};

/**
 * @brief PILZ规划配置结构
 */
struct PilzConfig {
    PilzPlanner planner_type;
    double velocity_scale;
    double acceleration_scale;
    double blending_radius;
    double max_deviation;           // 新增：最大位置偏差
    double orientation_tolerance;   // 新增：方向容差
    double goal_position_tolerance; // 新增：目标位置容差
    double goal_orientation_tolerance; // 新增：目标方向容差
    
    PilzConfig() 
        : planner_type(PilzPlanner::LIN), 
          velocity_scale(0.1),
          acceleration_scale(0.3),
          blending_radius(0.1),
          max_deviation(0.005),     // 1mm偏差
          orientation_tolerance(0.01), // 约0.58度
          goal_position_tolerance(0.005),  // 5mm
          goal_orientation_tolerance(0.01)  // 约0.57度
    {
    }
};

/**
 * @class CR7PilzPlanner
 * @brief PILZ工业规划器类
 * 
 * 这个类提供：
 * 1. PILZ规划器可用性检查
 * 2. PILZ规划器执行（LIN、PTP、CIRC）
 * 3. PILZ规划结果分析
 * 4. PILZ规划器配置管理
 */
class CR7PilzPlanner {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param move_group MoveGroup接口指针
     * @param logger 日志记录器
     */
    CR7PilzPlanner(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group
    );
    
    /**
     * @brief 析构函数
     */
    ~CR7PilzPlanner() = default;
    
    /**
     * @brief 检查PILZ规划器可用性
     * @return bool 是否可用
     */
    bool checkPilzAvailability() const;
    
    /**
     * @brief 获取可用的PILZ规划器列表
     * @return std::vector<std::string> 规划器列表
     */
    std::vector<std::string> getAvailablePilzPlanners() const;
    
    /**
     * @brief 设置PILZ规划器
     * @param planner_id 规划器ID
     * @return bool 设置成功返回true
     */
    bool setPilzPlanner(const std::string& planner_id);
    
    /**
     * @brief 测试PILZ规划器功能
     * @param planner_type 规划器类型
     * @return CR7BaseController::Result 测试结果
     */
    CR7BaseController::Result testPilzPlanner(PilzPlanner planner_type = PilzPlanner::LIN);
    
    /**
     * @brief 使用PILZ规划器执行直线运动
     * @param target_pose 目标位姿
     * @param config PILZ配置
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveWithPilzLin(
        const geometry_msgs::msg::Pose& target_pose,
        const PilzConfig& config = PilzConfig()
    );
    
    /**
     * @brief 使用PILZ规划器执行点对点运动
     * @param target_pose 目标位姿
     * @param config PILZ配置
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveWithPilzPtp(
        const geometry_msgs::msg::Pose& target_pose,
        const PilzConfig& config = PilzConfig()
    );
    
    /**
     * @brief 使用PILZ规划器执行圆周运动
     * @param intermediate_pose 中间点
     * @param target_pose 目标点
     * @param config PILZ配置
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveWithPilzCirc(
        const geometry_msgs::msg::Pose& intermediate_pose,
        const geometry_msgs::msg::Pose& target_pose,
        const PilzConfig& config = PilzConfig()
    );
    
    /**
     * @brief PILZ规划器类型转字符串
     * @param planner 规划器类型
     * @return std::string 规划器名称
     */
    static std::string pilzPlannerToString(PilzPlanner planner);
    
    /**
     * @brief 字符串转PILZ规划器类型
     * @param planner_name 规划器名称
     * @return PilzPlanner 规划器类型
     */
    static PilzPlanner stringToPilzPlanner(const std::string& planner_name);
    
    /**
     * @brief 获取PILZ规划配置
     * @return PilzConfig& 配置引用
     */
    PilzConfig& getConfig() 
    {
        return config_;
    }
    
private:
    /**
     * @brief 执行PILZ规划
     * @param target_pose 目标位姿
     * @param planner_id 规划器ID
     * @param config PILZ配置
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executePilzPlan(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& planner_id,
        const PilzConfig& config
    );
    
    /**
     * @brief 保存轨迹分析
     * @param trajectory 轨迹
     * @param filename_prefix 文件名前缀
     */
    void saveTrajectoryAnalysis(
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        const std::string& filename_prefix
    );
    
    /**
     * @brief 获取当前位姿
     * @return geometry_msgs::msg::Pose 当前位姿
     */
    geometry_msgs::msg::Pose getCurrentPose() const 
    {
        if (!move_group_)
        {
            return geometry_msgs::msg::Pose();
        }
        try
        {
            return move_group_->getCurrentPose().pose;
        }
        catch (...)
        {
            return geometry_msgs::msg::Pose();
        }
    }
    
    // 成员变量
    rclcpp::Node::SharedPtr node_;                      ///< ROS节点指针
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; ///< MoveGroup接口指针
    rclcpp::Logger logger_;                            ///< 日志记录器
    PilzConfig config_;                                ///< PILZ规划配置
};

} // namespace cr7_controller

#endif // CR7_PILZ_PLANNER_HPP_
