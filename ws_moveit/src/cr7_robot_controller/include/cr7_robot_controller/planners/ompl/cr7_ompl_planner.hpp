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
#include <moveit_visual_tools/moveit_visual_tools.h>

// 添加TF2头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"
#include "cr7_robot_controller/utils/trajectory_analyzer.hpp"

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
    double goal_joint_tolerance;     ///< 目标关节容差

    OMPLConfig() 
        : planner_id("RRTConnectkConfigDefault"),  // RRTConnect更适合约束规划
          planning_time(10.0),  // 减少规划时间
          num_planning_attempts(50),  // 减少尝试次数
          velocity_scale(0.2),
          acceleration_scale(0.5),
          goal_position_tolerance(0.001),
          goal_orientation_tolerance(0.001),
          goal_joint_tolerance(0.001)
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
    
    /**
     * @brief 执行带盒子约束的规划
     * @param target_pose 目标位姿
     * @param link_name 要约束的连杆名称
     * @param min_x 最小x坐标
     * @param max_x 最大x坐标
     * @param min_y 最小y坐标
     * @param max_y 最大y坐标
     * @param min_z 最小z坐标
     * @param max_z 最大z坐标
     * @param frame_id 参考坐标系
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithBoxConstraint(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& link_name,
        double min_x, double max_x,
        double min_y, double max_y,
        double min_z, double max_z,
        const std::string& frame_id = "base_link",
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 关节姿态枚举
     */
    enum class JointPoseType {
        SHOULDER_LEFT_ELBOW_UP_WRIST_NORMAL,    // 肩部左、肘部上、腕部正常
        SHOULDER_LEFT_ELBOW_UP_WRIST_FLIPPED,    // 肩部左、肘部上、腕部翻转
        SHOULDER_LEFT_ELBOW_DOWN_WRIST_NORMAL,  // 肩部左、肘部下、腕部正常
        SHOULDER_LEFT_ELBOW_DOWN_WRIST_FLIPPED,  // 肩部左、肘部下、腕部翻转
        SHOULDER_RIGHT_ELBOW_UP_WRIST_NORMAL,   // 肩部右、肘部上、腕部正常
        SHOULDER_RIGHT_ELBOW_UP_WRIST_FLIPPED,   // 肩部右、肘部上、腕部翻转
        SHOULDER_RIGHT_ELBOW_DOWN_WRIST_NORMAL, // 肩部右、肘部下、腕部正常
        SHOULDER_RIGHT_ELBOW_DOWN_WRIST_FLIPPED  // 肩部右、肘部下、腕部翻转
    };
    
    /**
     * @brief 设置关节姿态约束
     * @param pose_type 关节姿态类型
     */
    void setJointPoseConstraint(JointPoseType pose_type);
    
    /**
     * @brief 清除关节姿态约束
     */
    void clearJointPoseConstraints();
    
    /**
     * @brief 尝试不同的关节姿态规划
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithJointPoseVariations(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& waypoint_name);

    /**
     * @brief 关节角度范围结构体
     */
    struct JointRange {
        double min;  // 最小角度
        double max;  // 最大角度
        
        JointRange(double min_val = -M_PI, double max_val = M_PI) 
            : min(min_val), max(max_val) {}
    };
    
    /**
     * @brief 关节配置结构体
     */
    struct JointConfig {
        JointRange joint_1;  // 肩部旋转
        JointRange joint_2;  // 肩部俯仰
        JointRange joint_3;  // 肘部
        JointRange joint_4;  // 腕部旋转
        JointRange joint_5;  // 腕部俯仰
        JointRange joint_6;  // 腕部旋转
        
        // 默认构造函数，使用默认范围
        JointConfig() = default;
        
        // 根据关节姿态类型创建配置
        JointConfig(JointPoseType pose_type);
    };
    
    /**
     * @brief 通过IK解算多个关节配置并筛选规划
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithIKSolutions(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 通过IK解算多个关节配置并使用自定义关节范围筛选规划
     * @param target_pose 目标位姿
     * @param joint_config 关节配置范围
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithIKSolutions(
        const geometry_msgs::msg::Pose& target_pose,
        const JointConfig& joint_config,
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 通过IK解算多个关节配置并使用关节姿态类型筛选规划
     * @param target_pose 目标位姿
     * @param pose_type 关节姿态类型
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithIKSolutions(
        const geometry_msgs::msg::Pose& target_pose,
        JointPoseType pose_type,
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 执行带平面约束的规划
     * @param target_pose 目标位姿
     * @param link_name 要约束的连杆名称
     * @param plane_normal 平面法线
     * @param distance 平面距离原点的距离
     * @param frame_id 参考坐标系
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithPlaneConstraint(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& link_name,
        const geometry_msgs::msg::Vector3& plane_normal,
        double distance,
        const std::string& frame_id = "base_link",
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 执行带直线约束的规划
     * @param target_pose 目标位姿
     * @param link_name 要约束的连杆名称
     * @param line_start 直线起点
     * @param line_end 直线终点
     * @param frame_id 参考坐标系
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithLineConstraint(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& link_name,
        const geometry_msgs::msg::Point& line_start,
        const geometry_msgs::msg::Point& line_end,
        const std::string& frame_id = "base_link",
        const std::string& waypoint_name = ""
    );
    
    /**
     * @brief 执行带姿态约束的规划
     * @param target_pose 目标位姿
     * @param link_name 要约束的连杆名称
     * @param orientation 目标姿态
     * @param tolerance_x x轴方向容差
     * @param tolerance_y y轴方向容差
     * @param tolerance_z z轴方向容差
     * @param frame_id 参考坐标系
     * @param waypoint_name 路点名称（用于日志）
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result moveToPoseWithOrientationConstraint(
        const geometry_msgs::msg::Pose& target_pose,
        const std::string& link_name,
        const geometry_msgs::msg::Quaternion& orientation,
        double tolerance_x = 0.01,
        double tolerance_y = 0.01,
        double tolerance_z = 0.01,
        const std::string& frame_id = "base_link",
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
    
    // 添加MoveItVisualTools用于可视化
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_; ///< 可视化工具
};

}  // namespace cr7_controller

#endif  // CR7_OMPL_PLANNER_HPP_
