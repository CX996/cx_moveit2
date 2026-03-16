/**
 * @file trajectory_replanner.hpp
 * @brief 轨迹重规划工具类头文件
 * 
 * 这个文件定义了轨迹重规划相关的工具函数，包括：
 * 1. 轨迹时间参数化重规划
 * 2. 速度/加速度约束规划
 * 3. 路径参数化计算
 */

#ifndef TRAJECTORY_REPLANNER_HPP_
#define TRAJECTORY_REPLANNER_HPP_

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace cr7_controller {
namespace utils {

/**
 * @class TrajectoryReplanner
 * @brief 轨迹重规划工具类
 * 
 * 这个类提供轨迹重规划相关的静态工具函数
 */
class TrajectoryReplanner {
private:
    // 全局静态参数
    static double max_velocity_;     // 最大关节速度 (rad/s)
    static double max_acceleration_; // 最大关节加速度 (rad/s²)

public:
    /**
     * @brief 重新规划轨迹的时间特性（保持路径形状，改变速度和加速度）
     *
     * 核心思想：将原始轨迹看作一条几何路径，重新设计时间参数化
     * 支持两种模式：
     * 1. 指定总时间
     * 2. 指定最大速度和加速度约束
     *
     * @param input_traj 输入的关节轨迹
     * @param dt 输出轨迹的时间步长（秒），建议与底层控制频率匹配
     * @param mode 规划模式：0=指定总时间，1=指定最大速度/加速度
     * @param target_total_time 目标总时间（mode=0时有效）
     * @param max_velocity 最大速度（mode=1时有效，单位：路径参数/秒）
     * @param max_acceleration 最大加速度（mode=1时有效，单位：路径参数/秒²）
     * @param max_jerk 最大加加速度（可选，单位：路径参数/秒³）
     * @return 重新规划时间后的关节轨迹
     */
    static trajectory_msgs::msg::JointTrajectory reparameterizeTrajectory(
        const trajectory_msgs::msg::JointTrajectory& input_traj,
        double dt,
        int mode = 0,
        double target_total_time = 0.0,
        double max_velocity = 0.0,
        double max_acceleration = 0.0,
        double max_jerk = 0.0);

    /**
     * @brief 计算路径参数化 s∈[0,1]
     * @param traj 输入轨迹
     * @return 路径参数数组
     */
    static std::vector<double> calculatePathParameterization(
        const trajectory_msgs::msg::JointTrajectory& traj);

    /**
     * @brief 提取时间序列
     * @param traj 输入轨迹
     * @return 时间数组
     */
    static std::vector<double> extractTimes(
        const trajectory_msgs::msg::JointTrajectory& traj);

    /**
     * @brief 计算新的时间参数化（固定总时间）
     * @param original_times 原始时间序列
     * @param s_values 路径参数序列
     * @param target_total_time 目标总时间
     * @param new_times 输出新时间序列
     * @param new_s_values 输出新路径参数序列
     * @param dt 时间步长
     */
    static void calculateTimeParameterization_FixedTotalTime(
        const std::vector<double>& original_times,
        const std::vector<double>& s_values,
        double target_total_time,
        std::vector<double>& new_times,
        std::vector<double>& new_s_values,
        double dt);

    /**
     * @brief 计算新的时间参数化（S曲线速度规划）
     * @param original_times 原始时间序列
     * @param s_values 路径参数序列
     * @param max_velocity 最大速度
     * @param max_acceleration 最大加速度
     * @param max_jerk 最大加加速度
     * @param new_times 输出新时间序列
     * @param new_s_values 输出新路径参数序列
     * @param dt 时间步长
     */
    static void calculateTimeParameterization_VelAccConstraints(
        const std::vector<double>& original_times,
        const std::vector<double>& s_values,
        double max_velocity,
        double max_acceleration,
        double max_jerk,
        std::vector<double>& new_times,
        std::vector<double>& new_s_values,
        double dt);

    /**
     * @brief 计算新的时间参数化（保持原时间比例）
     * @param original_times 原始时间序列
     * @param s_values 路径参数序列
     * @param new_times 输出新时间序列
     * @param new_s_values 输出新路径参数序列
     * @param dt 时间步长
     */
    static void calculateTimeParameterization_PreserveRatio(
        const std::vector<double>& original_times,
        const std::vector<double>& s_values,
        std::vector<double>& new_times,
        std::vector<double>& new_s_values,
        double dt);

    /**
     * @brief 在s参数上插值
     * @param times 时间序列
     * @param s_values 路径参数序列
     * @param t 时间值
     * @return 插值后的路径参数值
     */
    static double interpolateS(
        const std::vector<double>& times,
        const std::vector<double>& s_values,
        double t);

    /**
     * @brief 在s参数上对轨迹点进行插值
     */
    static trajectory_msgs::msg::JointTrajectoryPoint interpolateTrajectoryPoint(
        const trajectory_msgs::msg::JointTrajectory& traj,
        const std::vector<double>& s_values,
        double s);

    /**
     * @brief 在新的时间点上采样轨迹
     * @param input_traj 输入轨迹
     * @param new_times 新时间序列
     * @param new_s_values 新路径参数序列
     * @param dt 时间步长
     * @return 重采样后的轨迹
     */
    static trajectory_msgs::msg::JointTrajectory resampleAtNewTimes(
        const trajectory_msgs::msg::JointTrajectory& input_traj,
        const std::vector<double>& new_times,
        const std::vector<double>& new_s_values,
        double dt);

    /**
     * @brief 使用五次多项式对关节轨迹进行重采样
     * 
     * 该函数使用五次多项式插值方法对输入轨迹进行重采样，生成指定时间步长的新轨迹。
     * 五次多项式能够保证位置、速度和加速度的连续性，提供平滑的轨迹过渡。
     * 
     * 特性：
     *  - 使用 Quintic Polynomial (五次多项式)
     *  - 保证 C2 连续 (位置、速度、加速度连续)
     *  - 保留原始轨迹的边界条件
     *  - 生成固定时间步长轨迹
     *  - 确保终点速度和加速度为0（机器人静止）
     * 
     * 适用于：
     *  - 工业机器人控制
     *  - ServoJ streaming
     *  - 轨迹控制周期匹配
     *  - 轨迹平滑处理
     * 
     * 算法原理：
     * 1. 对轨迹按时间步长 dt 进行均匀采样
     * 2. 对每个采样点，确定其所在的轨迹段
     * 3. 使用五次多项式插值计算该点的位置、速度和加速度
     * 4. 确保轨迹终点的速度和加速度为0
     * 
     * 五次多项式插值公式：
     * q(t) = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴ + c5*t⁵
     * v(t) = c1 + 2*c2*t + 3*c3*t² + 4*c4*t³ + 5*c5*t⁴
     * a(t) = 2*c2 + 6*c3*t + 12*c4*t² + 20*c5*t³
     * 
     * 其中系数 c0-c5 通过边界条件（起始和结束的位置、速度、加速度）计算得出
     * 
     * @param input_traj 输入的关节轨迹
     * @param dt 重采样的时间步长（秒），建议值：0.01-0.05秒
     * @return 重采样后的关节轨迹
     */
    static trajectory_msgs::msg::JointTrajectory resampleTrajectory(
        const trajectory_msgs::msg::JointTrajectory& input_traj,
        double dt);

    /**
     * @brief 设置全局最大关节速度
     * @param max_velocity 最大关节速度 (rad/s)
     */
    static void setMaxVelocity(double max_velocity);

    /**
     * @brief 设置全局最大关节加速度
     * @param max_acceleration 最大关节加速度 (rad/s²)
     */
    static void setMaxAcceleration(double max_acceleration);

    /**
     * @brief 获取全局最大关节速度
     * @return 最大关节速度 (rad/s)
     */
    static double getMaxVelocity();

    /**
     * @brief 获取全局最大关节加速度
     * @return 最大关节加速度 (rad/s²)
     */
    static double getMaxAcceleration();
};

} // namespace utils
} // namespace cr7_controller

#endif // TRAJECTORY_REPLANNER_HPP_