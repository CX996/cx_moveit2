/**
 * @file pose_utils.cpp
 * @brief 位姿处理工具类实现文件
 * 
 * 实现PoseUtils类的所有方法
 */

#include <cmath>
#include <tf2_ros/transform_listener.h>

#include "cr7_robot_controller/utils/pose_utils.hpp"

namespace cr7_controller {
namespace utils {

/**
 * @brief 转换位姿坐标
 */
bool PoseUtils::transformPose(const geometry_msgs::msg::PoseStamped& input_pose,
                            geometry_msgs::msg::PoseStamped& output_pose,
                            const std::string& target_frame,
                            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                            double timeout_sec,
                            rclcpp::Logger logger) {
    try 
    {
        // 等待坐标变换可用
        if (!tf_buffer->canTransform(target_frame, input_pose.header.frame_id,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(timeout_sec))) 
        {
            RCLCPP_ERROR(logger, "无法获取从 %s 到 %s 的坐标变换",
                        input_pose.header.frame_id.c_str(), target_frame.c_str());
            return false;
        }
        
        // 执行坐标变换
        output_pose = tf_buffer->transform(input_pose, target_frame);
        return true;
        
    } 
    catch (tf2::TransformException& ex) 
    {
        RCLCPP_ERROR(logger, "坐标变换失败: %s", ex.what());
        return false;
    }
}

/**
 * @brief 验证位姿有效性
 */
bool PoseUtils::validatePose(const geometry_msgs::msg::Pose& pose, 
                            std::string* error_msg) {
    // 检查位置是否有效
    if (std::isnan(pose.position.x) || std::isinf(pose.position.x) ||
        std::isnan(pose.position.y) || std::isinf(pose.position.y) ||
        std::isnan(pose.position.z) || std::isinf(pose.position.z)) 
    {
        if (error_msg) 
        {
            *error_msg = "位置包含无效值";
        }
        return false;
    }
    
    // 检查四元数是否有效
    if (std::isnan(pose.orientation.x) || std::isinf(pose.orientation.x) ||
        std::isnan(pose.orientation.y) || std::isinf(pose.orientation.y) ||
        std::isnan(pose.orientation.z) || std::isinf(pose.orientation.z) ||
        std::isnan(pose.orientation.w) || std::isinf(pose.orientation.w)) 
    {
        if (error_msg) 
        {
            *error_msg = "四元数包含无效值";
        }
        return false;
    }
    
    // 检查四元数是否为单位四元数
    double norm = pose.orientation.x*pose.orientation.x + 
                  pose.orientation.y*pose.orientation.y + 
                  pose.orientation.z*pose.orientation.z + 
                  pose.orientation.w*pose.orientation.w;
    
    if (std::abs(norm - 1.0) > 0.1) 
    {
        if (error_msg) 
        {
            *error_msg = "四元数不是单位四元数";
        }
        return false;
    }
    
    return true;
}

/**
 * @brief 结果转字符串
 */
std::string PoseUtils::resultToString(CR7BaseController::Result result) {
    switch (result) {
        case CR7BaseController::Result::SUCCESS: return "成功";
        case CR7BaseController::Result::PLANNING_FAILED: return "规划失败";
        case CR7BaseController::Result::EXECUTION_FAILED: return "执行失败";
        case CR7BaseController::Result::INVALID_INPUT: return "输入无效";
        case CR7BaseController::Result::ROBOT_NOT_READY: return "机器人未就绪";
        case CR7BaseController::Result::TIMEOUT: return "超时";
        case CR7BaseController::Result::CARTESIAN_FAILED: return "笛卡尔规划失败";
        default: return "未知结果";
    }
}

/**
 * @brief 打印位姿信息
 */
void PoseUtils::printPoseInfo(const geometry_msgs::msg::Pose& pose,
                            rclcpp::Logger logger,
                            const std::string& title) {
    RCLCPP_INFO(logger, "=====================================");
    RCLCPP_INFO(logger, "%s", title.c_str());
    RCLCPP_INFO(logger, "=====================================");
    RCLCPP_INFO(logger, "位置: [%.3f, %.3f, %.3f]", 
                pose.position.x, pose.position.y, pose.position.z);
    RCLCPP_INFO(logger, "姿态: [%.3f, %.3f, %.3f, %.3f]",
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w);
    RCLCPP_INFO(logger, "=====================================");
}

} // namespace utils
} // namespace cr7_controller
