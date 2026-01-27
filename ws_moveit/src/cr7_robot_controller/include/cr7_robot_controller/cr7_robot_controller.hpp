/**
 * @file cr7_robot_controller.hpp
 * @brief CR7机器人控制器头文件
 * 
 * 这个文件定义了CR7机器人控制器的核心类，包括：
 * 1. 机器人状态监控
 * 2. 运动规划执行
 * 3. 多路点控制
 * 4. 笛卡尔路径规划
 */

#ifndef CR7_ROBOT_CONTROLLER_HPP_
#define CR7_ROBOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace cr7_controller {

/**
 * @struct Waypoint
 * @brief 路点数据结构
 * 
 * 包含位置、姿态和名称，用于定义机器人需要到达的目标点
 */
struct Waypoint {
    std::string name;                      ///< 路点名称
    double x, y, z;                        ///< 位置 (米)
    double qx, qy, qz, qw;                 ///< 姿态 (四元数)
    
    /**
     * @brief 构造函数
     * @param name_ 路点名称
     * @param x_ X坐标
     * @param y_ Y坐标
     * @param z_ Z坐标
     * @param qx_ 四元数X分量
     * @param qy_ 四元数Y分量
     * @param qz_ 四元数Z分量
     * @param qw_ 四元数W分量
     */
    Waypoint(const std::string& name_ = "",
             double x_ = 0.0, double y_ = 0.0, double z_ = 0.0,
             double qx_ = 0.0, double qy_ = 0.0, double qz_ = 0.0, double qw_ = 1.0)
        : name(name_), x(x_), y(y_), z(z_), qx(qx_), qy(qy_), qz(qz_), qw(qw_) {}
    
    /**
     * @brief 转换为ROS Pose消息
     * @return geometry_msgs::msg::Pose ROS位姿消息
     */
    geometry_msgs::msg::Pose toPose() const;
    
    /**
     * @brief 验证四元数是否有效
     * @return bool 如果四元数是单位四元数则返回true
     */
    bool isValidQuaternion() const;
    
    /**
     * @brief 归一化四元数
     * 
     * 如果四元数不是单位四元数，将其归一化
     */
    void normalizeQuaternion();
};

/**
 * @class CR7RobotController
 * @brief CR7机器人控制器主类
 * 
 * 这个类封装了MoveIt的控制功能，提供：
 * 1. 单个位姿运动控制
 * 2. 多路点序列控制
 * 3. 笛卡尔路径规划
 * 4. 机器人状态监控
 * 5. 错误处理
 */
class CR7RobotController {
public:
    /**
     * @brief 规划结果枚举
     */
    enum class Result {
        SUCCESS,            ///< 规划执行成功
        PLANNING_FAILED,    ///< 规划失败
        EXECUTION_FAILED,   ///< 执行失败
        INVALID_INPUT,      ///< 输入无效
        ROBOT_NOT_READY,    ///< 机器人未就绪
        TIMEOUT,            ///< 超时
        CARTESIAN_FAILED    ///< 笛卡尔规划失败
    };
    
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
            : max_step(0.001), jump_threshold(1.5), eef_step(0.001), 
              avoid_collisions(true), max_velocity_factor(0.3) {}
    };
    
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param planning_group MoveIt规划组名称
     */
    CR7RobotController(rclcpp::Node::SharedPtr node, 
                      const std::string& planning_group = "cr7_group");
    
    /**
     * @brief 初始化控制器
     * @param timeout_sec 初始化超时时间(秒)
     * @return bool 初始化成功返回true
     * 
     * 等待机器人状态就绪，初始化MoveIt接口
     */
    bool initialize(double timeout_sec = 10.0);
    
    /**
     * @brief 移动到单个位姿
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return Result 规划结果
     */
    Result moveToPose(const geometry_msgs::msg::Pose& target_pose,
                     const std::string& waypoint_name = "");
    
    /**
     * @brief 移动到路点
     * @param waypoint 目标路点
     * @return Result 规划结果
     */
    Result moveToWaypoint(const Waypoint& waypoint);
    
    /**
     * @brief 执行多路点序列
     * @param waypoints 路点向量
     * @param delay_seconds 路点间延迟(秒)
     * @return std::vector<Result> 每个路点的结果
     */
    std::vector<Result> executeWaypoints(const std::vector<Waypoint>& waypoints,
                                        double delay_seconds = 2.0);
    
    /**
     * @brief 笛卡尔路径规划
     * @param waypoints 路径点序列
     * @param config 笛卡尔路径配置
     * @return Result 规划结果
     */
    Result executeCartesianPath(const std::vector<Waypoint>& waypoints,
                               const CartesianPathConfig& config);
    
    /**
     * @brief 执行预设焊接路径
     * @return std::vector<Result> 每个路点的结果
     */
    std::vector<Result> executeWeldingPath();
    
    /**
     * @brief 执行测试路径
     * @return std::vector<Result> 每个路点的结果
     */
    std::vector<Result> executeTestPath();
    
    /**
     * @brief 执行笛卡尔焊接路径
     * @return Result 规划结果
     */
    Result executeCartesianWeldingPath();
    
    /**
     * @brief 执行优化的笛卡尔路径（推荐使用）
     * @param waypoints 路径点序列
     * @return Result 规划结果
     */
    Result executeOptimizedCartesianPath(const std::vector<Waypoint>& waypoints);
    
    /**
     * @brief 检查机器人是否就绪
     * @return bool 机器人就绪返回true
     */
    bool isReady() const;
    
    /**
     * @brief 获取当前末端执行器位姿
     * @return geometry_msgs::msg::Pose 当前位姿
     */
    geometry_msgs::msg::Pose getCurrentPose() const;
    
    /**
     * @brief 获取当前关节位置
     * @return std::vector<double> 关节位置(弧度)
     */
    std::vector<double> getCurrentJointPositions() const;
    
    /**
     * @brief 打印当前状态
     */
    void printCurrentState() const;
    
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
     * @brief 获取笛卡尔路径配置
     * @return CartesianPathConfig& 配置引用
     */
    CartesianPathConfig& getCartesianConfig() { return cartesian_config_; }
    
    /**
     * @brief 结果转字符串（公开方法）
     * @param result 规划结果
     * @return std::string 结果描述
     */
    std::string resultToString(Result result) const;
    
private:
    // 原有的私有方法声明...
    bool waitForRobotState(double timeout_sec);
    bool validatePose(const geometry_msgs::msg::Pose& pose, 
                     std::string* error_msg = nullptr) const;
    bool preprocessWaypoints(const std::vector<Waypoint>& input_waypoints,
                           std::vector<geometry_msgs::msg::Pose>& output_poses,
                           const CartesianPathConfig& config);
    bool checkPathFeasibility(const std::vector<geometry_msgs::msg::Pose>& waypoints);
    double calculateDistance(const geometry_msgs::msg::Pose& pose1,
                           const geometry_msgs::msg::Pose& pose2) const;
    bool insertIntermediatePoints(const geometry_msgs::msg::Pose& start,
                                const geometry_msgs::msg::Pose& end,
                                std::vector<geometry_msgs::msg::Pose>& waypoints,
                                double max_step);
    bool slerpQuaternion(const geometry_msgs::msg::Quaternion& q1,
                        const geometry_msgs::msg::Quaternion& q2,
                        double t, geometry_msgs::msg::Quaternion& result) const;
    double trySimplifiedPath(const std::vector<geometry_msgs::msg::Pose>& original_waypoints,
                           moveit_msgs::msg::RobotTrajectory& trajectory,
                           const CartesianPathConfig& config);
    bool optimizeTrajectory(moveit_msgs::msg::RobotTrajectory& trajectory);
    bool isPointNecessary(const trajectory_msgs::msg::JointTrajectoryPoint& prev,
                        const trajectory_msgs::msg::JointTrajectoryPoint& current,
                        const trajectory_msgs::msg::JointTrajectoryPoint& next) const;
    bool validateTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory);
    
    // 新增的私有方法声明
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
     * @brief 打印轨迹点信息
     */
    void printTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& point,
                            size_t index);
    
    rclcpp::Node::SharedPtr node_;                          ///< ROS节点
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; ///< MoveIt接口
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_; ///< 规划场景接口
    bool initialized_;                                      ///< 初始化标志
    rclcpp::Logger logger_;                                 ///< 日志记录器
    CartesianPathConfig cartesian_config_;                 ///< 笛卡尔路径配置
};

}  // namespace cr7_controller

#endif  // CR7_ROBOT_CONTROLLER_HPP_