#include <vector>
#include <cmath>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/duration.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>


#include "cr7_robot_controller/utils/trajectory_replanner.hpp"

namespace cr7_controller {
namespace utils {

namespace {
static double toSec(const builtin_interfaces::msg::Duration& t)
{
    return static_cast<double>(t.sec) +
           static_cast<double>(t.nanosec) * 1e-9;
}
}

// 构造函数
TrajectoryReplanner::TrajectoryReplanner(const std::string& config_path)
    : velocity_scaling_factor_(0.1),
      acceleration_scaling_factor_(0.1),
      limits_loaded_(false)
{
    if (!config_path.empty()) 
    {
        loadJointLimits(config_path);
    }
}

// 加载关节限制配置文件
bool TrajectoryReplanner::loadJointLimits(const std::string& config_path)
{
    try {
        YAML::Node config = YAML::LoadFile(config_path);
        
        // 加载缩放因子
        if (config["default_velocity_scaling_factor"]) {
            velocity_scaling_factor_ = config["default_velocity_scaling_factor"].as<double>();
        }
        if (config["default_acceleration_scaling_factor"]) {
            acceleration_scaling_factor_ = config["default_acceleration_scaling_factor"].as<double>();
        }
        
        // 加载关节限制
        if (config["joint_limits"]) 
        {
            const YAML::Node& joint_limits = config["joint_limits"];
            for (const auto& joint : joint_limits) {
                const std::string& joint_name = joint.first.as<std::string>();
                const YAML::Node& limits = joint.second;
                
                if (limits["has_velocity_limits"] && limits["has_velocity_limits"].as<bool>()) 
                {
                    if (limits["max_velocity"]) 
                    {
                        joint_velocities_[joint_name] = limits["max_velocity"].as<double>();
                    }
                }
                
                if (limits["has_acceleration_limits"] && limits["has_acceleration_limits"].as<bool>()) 
                {
                    if (limits["max_acceleration"]) 
                    {
                        joint_accelerations_[joint_name] = limits["max_acceleration"].as<double>();
                    }
                }
            }
        }
        
        limits_loaded_ = true;
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("TrajectoryReplanner"), "Failed to load joint limits: %s", e.what());
        return false;
    }
}

// 设置速度缩放因子
void TrajectoryReplanner::setVelocityScalingFactor(double scaling_factor)
{
    if (scaling_factor >= 0.0 && scaling_factor <= 1.0) {
        velocity_scaling_factor_ = scaling_factor;
    }
}

// 设置加速度缩放因子
void TrajectoryReplanner::setAccelerationScalingFactor(double scaling_factor)
{
    if (scaling_factor >= 0.0 && scaling_factor <= 1.0) {
        acceleration_scaling_factor_ = scaling_factor;
    }
}

// 获取速度缩放因子
double TrajectoryReplanner::getVelocityScalingFactor() const
{
    return velocity_scaling_factor_;
}

// 获取加速度缩放因子
double TrajectoryReplanner::getAccelerationScalingFactor() const
{
    return acceleration_scaling_factor_;
}

// 获取关节最大速度
double TrajectoryReplanner::getJointMaxVelocity(const std::string& joint_name) const
{
    auto it = joint_velocities_.find(joint_name);
    if (it != joint_velocities_.end()) 
    {
        return it->second * velocity_scaling_factor_;
    }
    // 默认值
    return 3.0 * velocity_scaling_factor_;
}

// 获取关节最大加速度
double TrajectoryReplanner::getJointMaxAcceleration(const std::string& joint_name) const
{
    auto it = joint_accelerations_.find(joint_name);
    if (it != joint_accelerations_.end()) 
    {
        return it->second * acceleration_scaling_factor_;
    }
    // 默认值
    return 10.0 * acceleration_scaling_factor_;
}

/**
 * @brief S曲线规划器类
 */
class SCurvePlanner {
private:
    double max_velocity_ = 1.0;
    double max_acceleration_ = 1.0;
    double max_jerk_ = 0.0;
    double total_time_ = 0.0;
    double distance_ = 0.0;

public:
    void setMaxVelocity(double v) { max_velocity_ = v; }
    void setMaxAcceleration(double a) { max_acceleration_ = a; }
    void setMaxJerk(double j) { max_jerk_ = j; }
    
    double getTotalTime() const { return total_time_; }
    
    void plan(double start, double end, double start_vel, double end_vel)
    {
        distance_ = fabs(end - start);
        
        // 简化的S曲线规划（实际应用中需要更复杂的计算）
        // 这里使用梯形速度曲线作为简化版
        double time_to_max_vel = max_velocity_ / max_acceleration_;
        double distance_to_max_vel = 0.5 * max_acceleration_ * time_to_max_vel * time_to_max_vel;
        
        if (2 * distance_to_max_vel >= distance_) {
            // 三角形速度曲线
            double time_to_peak = sqrt(distance_ / max_acceleration_);
            total_time_ = 2 * time_to_peak;
        } else {
            // 梯形速度曲线
            double cruise_distance = distance_ - 2 * distance_to_max_vel;
            double cruise_time = cruise_distance / max_velocity_;
            total_time_ = 2 * time_to_max_vel + cruise_time;
        }
    }
    
    double getPosition(double t) const
    {
        if (t <= 0) return 0.0;
        if (t >= total_time_) return distance_;
        
        double time_to_max_vel = max_velocity_ / max_acceleration_;
        double distance_to_max_vel = 0.5 * max_acceleration_ * time_to_max_vel * time_to_max_vel;
        
        if (2 * distance_to_max_vel >= distance_) 
        {
            // 三角形速度曲线
            double time_to_peak = sqrt(distance_ / max_acceleration_);
            if (t <= time_to_peak) {
                return 0.5 * max_acceleration_ * t * t;
            } else {
                double t_descend = t - time_to_peak;
                double peak_vel = max_acceleration_ * time_to_peak;
                return (peak_vel * t_descend - 0.5 * max_acceleration_ * t_descend * t_descend) + distance_ / 2.0;
            }
        } 
        else 
        {
            // 梯形速度曲线
            double cruise_distance = distance_ - 2 * distance_to_max_vel;
            double cruise_time = cruise_distance / max_velocity_;
            double cruise_start_time = time_to_max_vel;
            double cruise_end_time = time_to_max_vel + cruise_time;
            
            if (t <= cruise_start_time) {
                return 0.5 * max_acceleration_ * t * t;
            } else if (t <= cruise_end_time) {
                double t_cruise = t - cruise_start_time;
                return distance_to_max_vel + max_velocity_ * t_cruise;
            } else {
                double t_descend = t - cruise_end_time;
                double peak_vel = max_velocity_;
                return distance_to_max_vel + cruise_distance + peak_vel * t_descend - 0.5 * max_acceleration_ * t_descend * t_descend;
            }
        }
    }
};

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
 * @param cartesian_path_length 笛卡尔空间路径长度（可选，单位：米），如果提供则基于笛卡尔路径长度进行速度规划，否则基于关节空间路径长度
 * @return 重新规划时间后的关节轨迹
 */
trajectory_msgs::msg::JointTrajectory TrajectoryReplanner::reparameterizeTrajectory(
    const trajectory_msgs::msg::JointTrajectory& input_traj,
    double dt,
    int mode,
    double target_total_time,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    double cartesian_path_length)
{
    // 1. 边界检查
    if (input_traj.points.size() < 2)
        return input_traj;
    
    // 2. 提取路径几何信息（与时间解耦）
    // 计算路径参数s(t)，s∈[0,1]表示从起点到终点的完成度
    std::vector<double> s_values = calculatePathParameterization(input_traj);
    
    // 3. 构建原始时间-路径参数映射
    std::vector<double> original_times = extractTimes(input_traj);
    
    // 4. 计算新的时间参数化 s_new(t_new)
    std::vector<double> new_times;
    std::vector<double> new_s_values;
    
    if (mode == 0) 
    {
        // 模式0：指定总时间
        calculateTimeParameterization_FixedTotalTime(
            original_times, s_values, target_total_time,
            new_times, new_s_values, dt);
    }
    else if (mode == 1) 
    {
        // 模式1：指定速度/加速度约束
        // 计算实际路径长度
        double total_path_length;
        if (cartesian_path_length > 0)
        {
            // 使用用户提供的笛卡尔空间路径长度
            total_path_length = cartesian_path_length;
            RCLCPP_INFO(rclcpp::get_logger("TrajectoryReplanner"), "Using cartesian path length: %.3f meters", total_path_length);
        }
        else
        {
            // 使用关节空间路径长度
            total_path_length = calculatePathLength(input_traj);
            RCLCPP_INFO(rclcpp::get_logger("TrajectoryReplanner"), "Using joint space path length: %.3f radians", total_path_length);
        }
        calculateTimeParameterization_VelAccConstraints(
            original_times, s_values, max_velocity, max_acceleration, max_jerk,
            new_times, new_s_values, dt, total_path_length);
    }
    else 
    {
        // 默认：保持原时间比例
        calculateTimeParameterization_PreserveRatio(
            original_times, s_values, new_times, new_s_values, dt);
    }
    
    // 5. 在新的时间点上采样轨迹
    return resampleAtNewTimes(input_traj, new_times, new_s_values, dt);
}

/**
 * @brief 计算路径参数化 s∈[0,1]
 */
std::vector<double> TrajectoryReplanner::calculatePathParameterization(
    const trajectory_msgs::msg::JointTrajectory& traj)
{
    std::vector<double> s_values(traj.points.size());
    std::vector<double> cumulative_lengths(traj.points.size(), 0.0);
    
    // 计算各段的关节空间弧长
    for (size_t i = 1; i < traj.points.size(); i++) 
    {
        double segment_length = 0.0;
        for (size_t j = 0; j < traj.joint_names.size(); j++) 
        {
            double delta = traj.points[i].positions[j] - traj.points[i-1].positions[j];
            segment_length += delta * delta;
        }
        cumulative_lengths[i] = cumulative_lengths[i-1] + sqrt(segment_length);
    }
    
    // 归一化到[0,1]
    double total_length = cumulative_lengths.back();
    for (size_t i = 0; i < s_values.size(); i++) 
    {
        s_values[i] = (total_length > 0) ? cumulative_lengths[i] / total_length : 0.0;
    }
    
    return s_values;
}

/**
 * @brief 计算轨迹的实际路径长度
 */
double TrajectoryReplanner::calculatePathLength(
    const trajectory_msgs::msg::JointTrajectory& traj)
{
    double total_length = 0.0;
    
    // 计算各段的关节空间弧长
    for (size_t i = 1; i < traj.points.size(); i++) 
    {
        double segment_length = 0.0;
        for (size_t j = 0; j < traj.joint_names.size(); j++) 
        {
            double delta = traj.points[i].positions[j] - traj.points[i-1].positions[j];
            segment_length += delta * delta;
        }
        total_length += sqrt(segment_length);
    }
    
    return total_length;
}

/**
 * @brief 提取时间序列
 */
std::vector<double> TrajectoryReplanner::extractTimes(
    const trajectory_msgs::msg::JointTrajectory& traj)
{
    std::vector<double> times(traj.points.size());
    for (size_t i = 0; i < traj.points.size(); i++) 
    {
        times[i] = toSec(traj.points[i].time_from_start);
    }
    return times;
}

/**
 * @brief 计算新的时间参数化（固定总时间）
 */
void TrajectoryReplanner::calculateTimeParameterization_FixedTotalTime(
    const std::vector<double>& original_times,
    const std::vector<double>& s_values,
    double target_total_time,
    std::vector<double>& new_times,
    std::vector<double>& new_s_values,
    double dt)
{
    new_times.clear();
    new_s_values.clear();
    
    double original_total_time = original_times.back();
    
    // 线性时间缩放
    for (double t_new = 0.0; t_new <= target_total_time; t_new += dt) 
    {
        // 计算对应的原始时间
        double t_original = (t_new / target_total_time) * original_total_time;
        
        // 在原始路径中找到对应的s值
        double s = interpolateS(original_times, s_values, t_original);
        
        new_times.push_back(t_new);
        new_s_values.push_back(s);
    }
}

/**
 * @brief 计算新的时间参数化（S曲线速度规划）
 * 
 * 该方法使用S曲线规划器计算新的时间参数化，支持基于关节空间或笛卡尔空间的路径长度
 * 
 * @param original_times 原始时间序列
 * @param s_values 路径参数序列
 * @param max_velocity 最大速度（单位：米/秒或弧度/秒，取决于total_path_length的单位）
 * @param max_acceleration 最大加速度（单位：米/秒²或弧度/秒²，取决于total_path_length的单位）
 * @param max_jerk 最大加加速度（可选，单位：米/秒³或弧度/秒³，取决于total_path_length的单位）
 * @param new_times 输出新时间序列
 * @param new_s_values 输出新路径参数序列
 * @param dt 时间步长
 * @param total_path_length 路径长度（单位：米或弧度，取决于是否基于笛卡尔空间）
 */
void TrajectoryReplanner::calculateTimeParameterization_VelAccConstraints(
    const std::vector<double>& original_times,
    const std::vector<double>& s_values,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    std::vector<double>& new_times,
    std::vector<double>& new_s_values,
    double dt,
    double total_path_length)
{
    new_times.clear();
    new_s_values.clear();
    
    // 创建S曲线规划器
    SCurvePlanner planner;
    planner.setMaxVelocity(max_velocity);
    planner.setMaxAcceleration(max_acceleration);
    
    if (max_jerk > 0) {
        planner.setMaxJerk(max_jerk);
    }
    
    // 计算S曲线，使用实际路径长度
    planner.plan(0.0, total_path_length, 0.0, 0.0);  // 从0到实际路径长度，起始速度=0，终点速度=0
    
    double total_time = planner.getTotalTime();
    
    // 采样S曲线
    for (double t = 0.0; t <= total_time; t += dt) {
        double position = planner.getPosition(t);
        double s = (total_path_length > 0) ? position / total_path_length : 0.0;  // 转换回归一化的s值
        new_times.push_back(t);
        new_s_values.push_back(s);
    }
    
    RCLCPP_INFO(rclcpp::get_logger("TrajectoryReplanner"), "S-curve planning completed. Total time: %.3f seconds, Path length: %.3f, Points: %zu", 
                total_time, total_path_length, new_times.size());
}

/**
 * @brief 计算新的时间参数化（保持原时间比例）
 */
void TrajectoryReplanner::calculateTimeParameterization_PreserveRatio(
    const std::vector<double>& original_times,
    const std::vector<double>& s_values,
    std::vector<double>& new_times,
    std::vector<double>& new_s_values,
    double dt)
{
    new_times.clear();
    new_s_values.clear();
    
    double original_total_time = original_times.back();
    
    // 保持原时间比例，按新的时间步长重新采样
    for (double t = 0.0; t <= original_total_time; t += dt) 
    {
        double s = interpolateS(original_times, s_values, t);
        new_times.push_back(t);
        new_s_values.push_back(s);
    }
}

/**
 * @brief 在s参数上插值
 */
double TrajectoryReplanner::interpolateS(
    const std::vector<double>& times,
    const std::vector<double>& s_values,
    double t)
{
    if (t <= times.front()) return s_values.front();
    if (t >= times.back()) return s_values.back();
    
    // 找到t所在的区间
    for (size_t i = 0; i < times.size() - 1; i++) 
    {
        if (t >= times[i] && t <= times[i+1]) {
            double alpha = (t - times[i]) / (times[i+1] - times[i]);
            return s_values[i] + alpha * (s_values[i+1] - s_values[i]);
        }
    }
    
    return s_values.back();
}


/**
 * @brief 在轨迹上插值
 * @param traj 输入轨迹
 * @param s_values 轨迹上的s参数序列
 * @param s 要插值的s参数值
 * @return 插值得到的轨迹点
 */
trajectory_msgs::msg::JointTrajectoryPoint TrajectoryReplanner::interpolateTrajectoryPoint(
    const trajectory_msgs::msg::JointTrajectory& traj,
    const std::vector<double>& s_values,
    double s)
{
    trajectory_msgs::msg::JointTrajectoryPoint result;

    // ================================
    // 1. 边界情况处理
    // ================================

    // 若采样点位于轨迹起点之前
    if (s < s_values.front())
    {
        auto p = traj.points.front();

        // 工业控制器通常要求轨迹起点速度和加速度为0
        std::fill(p.velocities.begin(), p.velocities.end(), 0.0);
        std::fill(p.accelerations.begin(), p.accelerations.end(), 0.0);

        RCLCPP_WARN(rclcpp::get_logger("TrajectoryReplanner"), "采样点位于轨迹起点之前，使用起点数据: s=%.3f", s);

        return p;
    }

    // 若采样点位于轨迹终点之后
    if (s > s_values.back())
    {
        auto p = traj.points.back();

        // 强制末端停止
        std::fill(p.velocities.begin(), p.velocities.end(), 0.0);
        std::fill(p.accelerations.begin(), p.accelerations.end(), 0.0);

        RCLCPP_WARN(rclcpp::get_logger("TrajectoryReplanner"), "采样点位于轨迹终点之后，使用终点数据: s=%.3f", s);

        return p;
    }

    // ================================
    // 2. 查找当前s所在的轨迹段
    // ================================

    size_t segment = 0;

    for (size_t i = 0; i < s_values.size() - 1; i++)
    {
        if (s >= s_values[i] && s <= s_values[i + 1])
        {
            segment = i;
            break;
        }
    }

    const auto& p0 = traj.points[segment];
    const auto& p1 = traj.points[segment + 1];

    double s0 = s_values[segment];
    double s1 = s_values[segment + 1];

    double t0 = toSec(p0.time_from_start);
    double t1 = toSec(p1.time_from_start);

    double delta_s = s1 - s0;
    double delta_t = t1 - t0;

    // ================================
    // 3. 极小路径段保护
    // ================================
    // 如果路径段非常短，五次多项式会导致数值不稳定
    // 工业机器人通常直接退化为线性插值

    if (delta_s < 1e-5 || delta_t < 1e-5)
    {
        trajectory_msgs::msg::JointTrajectoryPoint p = p0;

        double alpha = (s - s0) / (s1 - s0);

        size_t dof = p0.positions.size();

        p.positions.resize(dof);

        for (size_t j = 0; j < dof; j++)
        {
            p.positions[j] =
                p0.positions[j] +
                alpha * (p1.positions[j] - p0.positions[j]);
        }

        p.velocities.assign(dof, 0.0);
        p.accelerations.assign(dof, 0.0);

        RCLCPP_WARN(rclcpp::get_logger("TrajectoryReplanner"), "路径段过短，退化为线性插值: delta_s=%.6f, delta_t=%.6f", delta_s, delta_t);

        return p;
    }

    // ================================
    // 4. 计算ds/dt
    // ================================

    double ds_dt = delta_s / delta_t;

    // 当前s在本段中的局部变量
    double tau = s - s0;
    double T = delta_s;

    size_t dof = p0.positions.size();

    result.positions.resize(dof);
    result.velocities.resize(dof);
    result.accelerations.resize(dof);

    // ================================
    // 5. 速度、加速度限制
    // ================================
    // 工业机器人控制器必须限制速度和加速度，
    // 防止插值放大导致轨迹不可执行

    // ================================
    // 6. 对每个关节进行五次多项式插值
    // ================================

    for (size_t j = 0; j < dof; j++)
    {
        double q0 = p0.positions[j];
        double q1 = p1.positions[j];

        double v0_t = p0.velocities.empty() ? 0.0 : p0.velocities[j];
        double v1_t = p1.velocities.empty() ? 0.0 : p1.velocities[j];

        double a0_t = p0.accelerations.empty() ? 0.0 : p0.accelerations[j];
        double a1_t = p1.accelerations.empty() ? 0.0 : p1.accelerations[j];

        // ------------------------------
        // 转换为路径参数域
        // dq/dt -> dq/ds
        // ------------------------------

        double v0 = v0_t / ds_dt;
        double v1 = v1_t / ds_dt;

        double a0 = a0_t / (ds_dt * ds_dt);
        double a1 = a1_t / (ds_dt * ds_dt);

        // ------------------------------
        // quintic 多项式系数
        // ------------------------------

        double T2 = T*T;
        double T3 = T2*T;
        double T4 = T3*T;
        double T5 = T4*T;

        double c0 = q0;
        double c1 = v0;
        double c2 = a0 / 2.0;

        double c3 =
            (20*(q1-q0) - (8*v1+12*v0)*T - (3*a0-a1)*T2) / (2*T3);

        double c4 =
            (30*(q0-q1) + (14*v1+16*v0)*T + (3*a0-2*a1)*T2) / (2*T4);

        double c5 =
            (12*(q1-q0) - (6*v1+6*v0)*T - (a0-a1)*T2) / (2*T5);

        double tau2 = tau*tau;
        double tau3 = tau2*tau;
        double tau4 = tau3*tau;
        double tau5 = tau4*tau;

        // ------------------------------
        // 位置
        // ------------------------------

        result.positions[j] =
            c0 +
            c1*tau +
            c2*tau2 +
            c3*tau3 +
            c4*tau4 +
            c5*tau5;

        // ------------------------------
        // dq/ds
        // ------------------------------

        double dq_ds =
            c1 +
            2*c2*tau +
            3*c3*tau2 +
            4*c4*tau3 +
            5*c5*tau4;

        // ------------------------------
        // d²q/ds²
        // ------------------------------

        double d2q_ds2 =
            2*c2 +
            6*c3*tau +
            12*c4*tau2 +
            20*c5*tau3;

        // ------------------------------
        // 转回时间域
        // ------------------------------

        double vel = dq_ds * ds_dt;
        double acc = d2q_ds2 * ds_dt * ds_dt;

        // ------------------------------
        // 速度限幅
        // ------------------------------

        // 获取关节特定的速度限制
        double joint_max_vel = 3.0; // 默认值
        if (j < traj.joint_names.size()) {
            joint_max_vel = getJointMaxVelocity(traj.joint_names[j]);
        }
        if (vel > joint_max_vel) vel = joint_max_vel;
        if (vel < -joint_max_vel) vel = -joint_max_vel;

        // ------------------------------
        // 加速度限幅
        // ------------------------------

        // 获取关节特定的加速度限制
        double joint_max_acc = 10.0; // 默认值
        if (j < traj.joint_names.size()) {
            joint_max_acc = getJointMaxAcceleration(traj.joint_names[j]);
        }
        if (acc > joint_max_acc) acc = joint_max_acc;
        if (acc < -joint_max_acc) acc = -joint_max_acc;

        result.velocities[j] = vel;
        result.accelerations[j] = acc;
    }

    return result;
}


/**
 * @brief 在新的时间点上采样轨迹
 */
trajectory_msgs::msg::JointTrajectory TrajectoryReplanner::resampleAtNewTimes(
    const trajectory_msgs::msg::JointTrajectory& input_traj,
    const std::vector<double>& new_times,
    const std::vector<double>& new_s_values,
    double dt)
{
    trajectory_msgs::msg::JointTrajectory output;
    output.joint_names = input_traj.joint_names;
    
    // 提取原始s值
    std::vector<double> original_s = calculatePathParameterization(input_traj);
    
    // 在新的时间点上采样
    for (size_t i = 0; i < new_times.size(); i++) 
    {
        double s = new_s_values[i];
        trajectory_msgs::msg::JointTrajectoryPoint point = interpolateTrajectoryPoint(input_traj, original_s, s);
        point.time_from_start = rclcpp::Duration::from_seconds(new_times[i]);
        output.points.push_back(point);
    }
    
    // 确保终点速度和加速度为0
    if (!output.points.empty()) {
        trajectory_msgs::msg::JointTrajectoryPoint& end_point = output.points.back();
        size_t dof = end_point.positions.size();
        
        if (!end_point.velocities.empty()) {
            for (size_t j = 0; j < dof; j++) {
                end_point.velocities[j] = 0.0;
            }
        }
        
        if (!end_point.accelerations.empty()) {
            for (size_t j = 0; j < dof; j++) {
                end_point.accelerations[j] = 0.0;
            }
        }
    }
    
    return output;
}

/**
 * @brief 使用五次多项式对关节轨迹进行重采样
 */
trajectory_msgs::msg::JointTrajectory TrajectoryReplanner::resampleTrajectory(
    const trajectory_msgs::msg::JointTrajectory& input_traj,
    double dt)
{
    // 创建输出轨迹
    trajectory_msgs::msg::JointTrajectory output;

    // 处理边界情况：轨迹点少于2个时直接返回原轨迹
    if (input_traj.points.size() < 2)
        return input_traj;

    // 复制关节名称信息
    output.joint_names = input_traj.joint_names;

    // 计算轨迹总时间
    double total_time = toSec(input_traj.points.back().time_from_start);

    // 当前轨迹段索引
    size_t segment = 0;

    // 按指定时间步长逐点采样
    for (double t = 0.0; t < total_time; t += dt)
    {
        // 找到当前时间所在的轨迹段
        // 注意：使用 input_traj.points.size() - 2 作为上界，确保 segment + 1 不会越界
        while (segment < input_traj.points.size() - 2 &&
               t > toSec(input_traj.points[segment + 1].time_from_start))
        {
            segment++;
        }

        // 获取当前轨迹段的起点和终点
        const auto& p0 = input_traj.points[segment];
        const auto& p1 = input_traj.points[segment + 1];

        // 计算当前轨迹段的时间信息
        double t0 = toSec(p0.time_from_start);  // 轨迹段起始时间
        double t1 = toSec(p1.time_from_start);  // 轨迹段结束时间
        double T = t1 - t0;                     // 轨迹段持续时间
        double tau = t - t0;                    // 当前时间在轨迹段内的相对时间

        // 创建新的轨迹点
        trajectory_msgs::msg::JointTrajectoryPoint new_point;

        // 获取自由度数量
        size_t dof = p0.positions.size();

        // 为新轨迹点分配空间
        new_point.positions.resize(dof);
        new_point.velocities.resize(dof);
        new_point.accelerations.resize(dof);

        // 对每个关节进行五次多项式插值
        for (size_t j = 0; j < dof; j++)
        {
            // 提取边界条件
            double q0 = p0.positions[j];          // 起始位置
            double q1 = p1.positions[j];          // 结束位置
            double v0 = p0.velocities.empty() ? 0.0 : p0.velocities[j];  // 起始速度
            double v1 = p1.velocities.empty() ? 0.0 : p1.velocities[j];  // 结束速度
            double a0 = p0.accelerations.empty() ? 0.0 : p0.accelerations[j];  // 起始加速度
            double a1 = p1.accelerations.empty() ? 0.0 : p1.accelerations[j];  // 结束加速度

            // 计算五次多项式系数
            double c0 = q0;                          // 常数项（起始位置）
            double c1 = v0;                          // 一次项系数（起始速度）
            double c2 = a0 / 2.0;                    // 二次项系数（起始加速度）

            // 预计算时间相关的幂次，提高计算效率
            double T2 = T*T;
            double T3 = T2*T;
            double T4 = T3*T;
            double T5 = T4*T;

            // 计算三次项系数
            double c3 =
                (20*(q1-q0) - (8*v1+12*v0)*T - (3*a0-a1)*T2) / (2*T3);

            // 计算四次项系数
            double c4 =
                (30*(q0-q1) + (14*v1+16*v0)*T + (3*a0-2*a1)*T2) / (2*T4);

            // 计算五次项系数
            double c5 =
                (12*(q1-q0) - (6*v1+6*v0)*T - (a0-a1)*T2) / (2*T5);

            // 预计算相对时间的幂次，提高计算效率
            double tau2 = tau*tau;
            double tau3 = tau2*tau;
            double tau4 = tau3*tau;
            double tau5 = tau4*tau;

            // 计算当前时间点的位置
            new_point.positions[j] =
                c0 +
                c1*tau +
                c2*tau2 +
                c3*tau3 +
                c4*tau4 +
                c5*tau5;

            // 计算当前时间点的速度（位置对时间的一阶导数）
            new_point.velocities[j] =
                c1 +
                2*c2*tau +
                3*c3*tau2 +
                4*c4*tau3 +
                5*c5*tau4;

            // 计算当前时间点的加速度（位置对时间的二阶导数）
            new_point.accelerations[j] =
                2*c2 +
                6*c3*tau +
                12*c4*tau2 +
                20*c5*tau3;
        }

        // 设置新轨迹点的时间戳
        new_point.time_from_start =
            rclcpp::Duration::from_seconds(t);

        // 将新轨迹点添加到输出轨迹中
        output.points.push_back(new_point);
    }

    // 确保输出轨迹的终点与输入轨迹完全一致
    // 这是为了避免由于浮点数计算误差导致的终点偏差
    // 同时确保终点的速度和加速度都为0（机器人应该在终点静止）
    trajectory_msgs::msg::JointTrajectoryPoint end_point = input_traj.points.back();
    for (size_t j = 0; j < end_point.positions.size(); j++)
    {
        if (!end_point.velocities.empty())
            end_point.velocities[j] = 0.0;
        if (!end_point.accelerations.empty())
            end_point.accelerations[j] = 0.0;
    }
    output.points.push_back(end_point);

    return output;
}



} // namespace utils
} // namespace cr7_controller

