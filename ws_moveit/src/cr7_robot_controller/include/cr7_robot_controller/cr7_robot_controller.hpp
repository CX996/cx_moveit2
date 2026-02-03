/**
 * @file cr7_robot_controller.hpp
 * @brief CR7机器人控制器头文件
 * 
 * 这个文件定义了CR7机器人控制器的核心类，包括：
 * 1. 机器人状态监控
 * 2. 运动规划执行
 * 3. 多路点控制
 * 4. 笛卡尔路径规划
 * 5. PILZ工业规划器集成
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

// 添加TF2相关头文件
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cr7_controller {

/**
 * @struct Waypoint
 * @brief 路点数据结构
 * 
 * 包含位置、姿态和名称，用于定义机器人需要到达的目标点
 */
struct Waypoint 
{
    std::string name;                      ///< 路点名称
    double x, y, z;                        ///< 位置 (米)
    double qx, qy, qz, qw;                 ///< 姿态 (四元数)
    std::string frame_id;  // 新增：目标坐标系

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
    Waypoint(const std::string& name, 
             double x, double y, double z,
             double qx, double qy, double qz, double qw,
             const std::string& frame_id = "base_link")  // 默认基坐标系
        : name(name), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw), 
          frame_id(frame_id) {}
    
    /**
     * @brief 转换为ROS PoseStamped消息
     * @return geometry_msgs::msg::PoseStamped ROS位姿消息
     */  
    geometry_msgs::msg::PoseStamped toPoseStamped() const;  // 新增方法

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
          avoid_collisions(true), max_velocity_factor(0.3) {}
};


/**
 * @class CR7RobotController
 * @brief CR7机器人控制器主类
 * 
 * 这个类封装了MoveIt的控制功能，提供：
 * 1. 单个位姿运动控制
 * 2. 多路点序列控制
 * 3. 笛卡尔路径规划
 * 4. PILZ工业规划器
 * 5. 机器人状态监控
 * 6. 错误处理
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
    // struct PilzConfig {
    //     PilzPlanner planner_type;   ///< 规划器类型
    //     double velocity_scale;      ///< 速度缩放因子
    //     double acceleration_scale;   ///< 加速度缩放因子
    //     double blending_radius;      ///< 轨迹混合半径
        
    //     PilzConfig() 
    //         : planner_type(PilzPlanner::LIN), 
    //           velocity_scale(0.1),
    //           acceleration_scale(0.3),
    //           blending_radius(0.1) {}
    // };

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
            goal_orientation_tolerance(0.01) {}  // 约0.57度
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
    
    // ============================================================================
    // 基础运动控制方法
    // ============================================================================
    
    bool transformPose(const geometry_msgs::msg::PoseStamped& input_pose,
                                        geometry_msgs::msg::PoseStamped& output_pose,
                                        const std::string& target_frame,
                                        double timeout_sec = 2.0);

    // 修改 moveToPose 方法，支持 PoseStamped
    Result moveToPose(const geometry_msgs::msg::PoseStamped& target_pose,
                    const std::string& waypoint_name);                        

    /**
     * @brief 移动到单个位姿
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return Result 规划结果
     */
    Result moveToPose(const geometry_msgs::msg::Pose& target_pose,
                    const std::string& waypoint_name);
    
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
    
    // ============================================================================
    // 笛卡尔路径规划方法
    // ============================================================================
    
    /**
     * @brief 笛卡尔路径规划
     * @param waypoints 路径点序列
     * @param config 笛卡尔路径配置
     * @return Result 规划结果
     */
    Result executeCartesianPath(const std::vector<Waypoint>& waypoints,
                               const CartesianPathConfig& config);
    
    /**
     * @brief 执行优化的笛卡尔路径（推荐使用）
     * @param waypoints 路径点序列
     * @return Result 规划结果
     */
    Result executeOptimizedCartesianPath(const std::vector<Waypoint>& waypoints);
    
    // ============================================================================
    // PILZ工业规划器方法
    // ============================================================================
    
    /**
     * @brief 测试PILZ规划器功能
     * @param planner_type 规划器类型
     * @return Result 测试结果
     */
    Result testPilzPlanner(PilzPlanner planner_type = PilzPlanner::LIN);
    
    /**
     * @brief 使用PILZ规划器执行直线运动
     * @param target_pose 目标位姿
     * @param config PILZ配置
     * @return Result 规划结果
     */
    Result moveWithPilzLin(const geometry_msgs::msg::Pose& target_pose,
                          const PilzConfig& config = PilzConfig());
    
    /**
     * @brief 使用PILZ规划器执行点对点运动
     * @param target_pose 目标位姿
     * @param config PILZ配置
     * @return Result 规划结果
     */
    Result moveWithPilzPtp(const geometry_msgs::msg::Pose& target_pose,
                          const PilzConfig& config = PilzConfig());
    
    /**
     * @brief 使用PILZ规划器执行圆周运动
     * @param intermediate_pose 中间点
     * @param target_pose 目标点
     * @param config PILZ配置
     * @return Result 规划结果
     */
    Result moveWithPilzCirc(const geometry_msgs::msg::Pose& intermediate_pose,
                           const geometry_msgs::msg::Pose& target_pose,
                           const PilzConfig& config = PilzConfig());
    
    /**
     * @brief 设置PILZ规划器参数
     * @param planner_id 规划器ID
     * @return bool 设置成功返回true
     */
    bool setPilzPlanner(const std::string& planner_id);
    
    /**
     * @brief 获取可用的PILZ规划器列表
     * @return std::vector<std::string> 规划器列表
     */
    std::vector<std::string> getAvailablePilzPlanners() const;
    
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
    
    // ============================================================================
    // 预设路径执行方法
    // ============================================================================
    
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
     * @brief 使用PILZ执行焊接点位路径
     * 先用关节空间移动到起点，再用PILZ执行到终点
     * @return Result 规划结果
     */
    Result executePilzWeldingPath();
    
    /**
     * @brief 使用工具坐标系进行点位执行
     * @return Result 规划结果
     */
    Result executetoolAxis() ;
    // ============================================================================
    // 状态查询和配置方法
    // ============================================================================
    
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
     * @brief 获取PILZ规划器配置
     * @return PilzConfig& 配置引用
     */
    PilzConfig& getPilzConfig() { return pilz_config_; }
    
    /**
     * @brief 结果转字符串（公开方法）
     * @param result 规划结果
     * @return std::string 结果描述
     */
    std::string resultToString(Result result) const;
    
private:
    // ============================================================================
    // 私有辅助方法
    // ============================================================================
    
    Result moveToPoseImpl(const geometry_msgs::msg::Pose& target_pose,
                         const std::string& waypoint_name);

    /**
     * @brief 等待机器人状态就绪
     */
    bool waitForRobotState(double timeout_sec);
    
    /**
     * @brief 验证位姿有效性
     */
    bool validatePose(const geometry_msgs::msg::Pose& pose, 
                     std::string* error_msg = nullptr) const;
    
    // ============================================================================
    // 笛卡尔路径规划私有方法
    // ============================================================================
    
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
     * @brief 打印轨迹点信息
     */
    void printTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& point,
                            size_t index);
    
    // ============================================================================
    // PILZ规划器私有方法
    // ============================================================================
    
    /**
     * @brief 验证PILZ规划器可用性
     * @return bool 如果PILZ规划器可用返回true
     */
    bool checkPilzAvailability() const;
    
    /**
     * @brief 执行PILZ规划
     */
    Result executePilzPlan(const geometry_msgs::msg::Pose& target_pose,
                          const std::string& planner_id,
                          const PilzConfig& config);
    
    // ============================================================================
    // 私有成员变量
    // ============================================================================
    
    rclcpp::Node::SharedPtr node_;                          ///< ROS节点
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; ///< MoveIt接口
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_; ///< 规划场景接口
    bool initialized_;                                      ///< 初始化标志
    rclcpp::Logger logger_;                                 ///< 日志记录器
    CartesianPathConfig cartesian_config_;                 ///< 笛卡尔路径配置
    PilzConfig pilz_config_;                               ///< PILZ规划器配置

    // 添加坐标变换监听器
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

}  // namespace cr7_controller

#endif  // CR7_ROBOT_CONTROLLER_HPP_