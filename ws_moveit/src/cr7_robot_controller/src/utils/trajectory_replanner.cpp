#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <builtin_interfaces/msg/duration.hpp>

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
        if (t >= total_time_) return 1.0;
        
        double time_to_max_vel = max_velocity_ / max_acceleration_;
        double distance_to_max_vel = 0.5 * max_acceleration_ * time_to_max_vel * time_to_max_vel;
        
        if (2 * distance_to_max_vel >= distance_) 
        {
            // 三角形速度曲线
            double time_to_peak = sqrt(distance_ / max_acceleration_);
            if (t <= time_to_peak) {
                return 0.5 * max_acceleration_ * t * t / distance_;
            } else {
                double t_descend = t - time_to_peak;
                double peak_vel = max_acceleration_ * time_to_peak;
                return (peak_vel * t_descend - 0.5 * max_acceleration_ * t_descend * t_descend) / distance_ + 0.5;
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
                return 0.5 * max_acceleration_ * t * t / distance_;
            } else if (t <= cruise_end_time) {
                double t_cruise = t - cruise_start_time;
                return (distance_to_max_vel + max_velocity_ * t_cruise) / distance_;
            } else {
                double t_descend = t - cruise_end_time;
                double peak_vel = max_velocity_;
                return (distance_to_max_vel + cruise_distance + peak_vel * t_descend - 0.5 * max_acceleration_ * t_descend * t_descend) / distance_;
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
 * @return 重新规划时间后的关节轨迹
 */
trajectory_msgs::msg::JointTrajectory TrajectoryReplanner::reparameterizeTrajectory(
    const trajectory_msgs::msg::JointTrajectory& input_traj,
    double dt,
    int mode,
    double target_total_time,
    double max_velocity,
    double max_acceleration,
    double max_jerk)
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
        calculateTimeParameterization_VelAccConstraints(
            original_times, s_values, max_velocity, max_acceleration, max_jerk,
            new_times, new_s_values, dt);
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
 */
void TrajectoryReplanner::calculateTimeParameterization_VelAccConstraints(
    const std::vector<double>& original_times,
    const std::vector<double>& s_values,
    double max_velocity,
    double max_acceleration,
    double max_jerk,
    std::vector<double>& new_times,
    std::vector<double>& new_s_values,
    double dt)
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
    
    // 计算S曲线
    planner.plan(0.0, 1.0, 0.0, 0.0);  // 从s=0到s=1，起始速度=0，终点速度=0
    
    double total_time = planner.getTotalTime();
    
    // 采样S曲线
    for (double t = 0.0; t <= total_time; t += dt) {
        double s = planner.getPosition(t);
        new_times.push_back(t);
        new_s_values.push_back(s);
    }
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
 * @brief 在s参数上对轨迹点进行插值
 */
trajectory_msgs::msg::JointTrajectoryPoint TrajectoryReplanner::interpolateTrajectoryPoint(
    const trajectory_msgs::msg::JointTrajectory& traj,
    const std::vector<double>& s_values,
    double s)
{
    trajectory_msgs::msg::JointTrajectoryPoint result;
    
    if (s <= s_values.front()) 
    {
        return traj.points.front();
    }
    if (s >= s_values.back()) 
    {
        return traj.points.back();
    }
    
    // 找到s所在的区间
    size_t segment = 0;
    for (size_t i = 0; i < s_values.size() - 1; i++) 
    {
        if (s >= s_values[i] && s <= s_values[i+1]) 
        {
            segment = i;
            break;
        }
    }
    
    // 获取当前轨迹段的起点和终点
    const auto& p0 = traj.points[segment];
    const auto& p1 = traj.points[segment + 1];
    double s0 = s_values[segment];  // 轨迹段起始路径参数
    double s1 = s_values[segment + 1];  // 轨迹段结束路径参数
    double T = s1 - s0;  // 轨迹段路径参数范围
    double tau = s - s0;  // 当前路径参数在轨迹段内的相对值
    
    size_t dof = p0.positions.size();
    result.positions.resize(dof);
    result.velocities.resize(dof);
    result.accelerations.resize(dof);
    
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

        // 预计算路径参数相关的幂次，提高计算效率
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

        // 预计算相对路径参数的幂次，提高计算效率
        double tau2 = tau*tau;
        double tau3 = tau2*tau;
        double tau4 = tau3*tau;
        double tau5 = tau4*tau;

        // 计算当前路径参数点的位置
        result.positions[j] =
            c0 +
            c1*tau +
            c2*tau2 +
            c3*tau3 +
            c4*tau4 +
            c5*tau5;

        // 计算当前路径参数点的速度（位置对路径参数的一阶导数）
        result.velocities[j] =
            c1 +
            2*c2*tau +
            3*c3*tau2 +
            4*c4*tau3 +
            5*c5*tau4;

        // 计算当前路径参数点的加速度（位置对路径参数的二阶导数）
        result.accelerations[j] =
            2*c2 +
            6*c3*tau +
            12*c4*tau2 +
            20*c5*tau3;
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

