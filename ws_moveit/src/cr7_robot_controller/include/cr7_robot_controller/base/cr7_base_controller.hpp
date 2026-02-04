/**
 * @file cr7_base_controller.hpp
 * @brief CR7机器人基础控制器类
 * 
 * 包含CR7机器人控制器的核心功能：
 * 1. 机器人状态监控
 * 2. 基础运动控制
 * 3. 路点管理
 * 4. 坐标变换
 */

#ifndef CR7_BASE_CONTROLLER_HPP_
#define CR7_BASE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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
    std::string frame_id;                  ///< 目标坐标系

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
        : name(name), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw), frame_id(frame_id) {}
    
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
 * @class CR7BaseController
 * @brief CR7机器人基础控制器类
 * 
 * 这个类封装了MoveIt的控制功能，提供：
 * 1. 机器人状态监控
 * 2. 单个位姿运动控制
 * 3. 多路点序列控制
 * 4. 坐标变换
 */
class CR7BaseController 
{
    
public:
    /**
     * @brief 规划结果枚举
     */
    enum class Result 
    {
        SUCCESS,            ///< 规划执行成功
        PLANNING_FAILED,    ///< 规划失败
        EXECUTION_FAILED,   ///< 执行失败
        INVALID_INPUT,      ///< 输入无效
        ROBOT_NOT_READY,    ///< 机器人未就绪
        TIMEOUT,            ///< 超时
        CARTESIAN_FAILED    ///< 笛卡尔规划失败
    };

    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param planning_group MoveIt规划组名称
     */
    CR7BaseController(rclcpp::Node::SharedPtr node, 
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
    
    /**
     * @brief 移动到单个位姿
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return Result 规划结果
     */
    Result moveToPose(const geometry_msgs::msg::Pose& target_pose,
                    const std::string& waypoint_name);

    /**
     * @brief 移动到单个位姿，修改 moveToPose 方法，支持 PoseStamped
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return Result 规划结果
     */
    Result moveToPose(const geometry_msgs::msg::PoseStamped& target_pose,
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
     * @brief 结果转字符串（公开方法）
     * @param result 规划结果
     * @return std::string 结果描述
     */
    std::string resultToString(Result result) const;
    
protected:
    // ============================================================================
    // 受保护成员变量
    // ============================================================================
    rclcpp::Node::SharedPtr node_;                          ///< ROS节点
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_; ///< MoveIt接口
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_; ///< 规划场景接口
    bool initialized_;                                      ///< 初始化标志
    rclcpp::Logger logger_;                                 ///< 日志记录器

    // 添加坐标变换监听器
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; ///< TF2缓冲区
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; ///< TF2监听器
    
    // ============================================================================
    // 私有辅助方法
    // ============================================================================
    /**
     * @brief 转换位姿坐标
     * @param input_pose 输入位姿
     * @param output_pose 输出位姿
     * @param target_frame 目标坐标系
     * @param timeout_sec 转换超时时间(秒)
     * @return bool 转换成功返回true
     */
    bool transformPose(const geometry_msgs::msg::PoseStamped& input_pose,
                                        geometry_msgs::msg::PoseStamped& output_pose,
                                        const std::string& target_frame,
                                        double timeout_sec = 2.0);

    /**
     * @brief 内部实现移动到位姿
     * @param target_pose 目标位姿
     * @param waypoint_name 路点名称（用于日志）
     * @return Result 规划结果
     */                                   
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
};

}  // namespace cr7_controller

#endif  // CR7_BASE_CONTROLLER_HPP_
