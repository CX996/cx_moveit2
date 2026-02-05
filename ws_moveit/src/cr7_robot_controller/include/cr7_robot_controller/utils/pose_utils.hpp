/**
 * @file pose_utils.hpp
 * @brief 位姿处理工具类头文件
 * 
 * 这个文件定义了位姿处理相关的工具函数，包括：
 * 1. 坐标变换
 * 2. 位姿验证
 * 3. 结果转换
 */

#ifndef POSE_UTILS_HPP_
#define POSE_UTILS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"

namespace cr7_controller {
namespace utils {

/**
 * @class PoseUtils
 * @brief 位姿处理工具类
 * 
 * 这个类提供位姿处理相关的静态工具函数
 */
class PoseUtils {
public:
    /**
     * @brief 转换位姿坐标
     * @param input_pose 输入位姿
     * @param output_pose 输出位姿
     * @param target_frame 目标坐标系
     * @param tf_buffer TF2缓冲区
     * @param timeout_sec 转换超时时间(秒)
     * @return bool 转换成功返回true
     */
    static bool transformPose(const geometry_msgs::msg::PoseStamped& input_pose,
                            geometry_msgs::msg::PoseStamped& output_pose,
                            const std::string& target_frame,
                            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                            double timeout_sec = 2.0,
                            rclcpp::Logger logger = rclcpp::get_logger("pose_utils"));
    
    /**
     * @brief 验证位姿有效性
     * @param pose 要验证的位姿
     * @param error_msg 错误信息输出
     * @return bool 位姿有效返回true
     */
    static bool validatePose(const geometry_msgs::msg::Pose& pose, 
                            std::string* error_msg = nullptr);
    
    /**
     * @brief 结果转字符串
     * @param result 规划结果
     * @return std::string 结果描述
     */
    static std::string resultToString(CR7BaseController::Result result);
    
    /**
     * @brief 打印位姿信息
     * @param pose 要打印的位姿
     * @param logger ROS logger
     * @param title 标题
     */
    static void printPoseInfo(const geometry_msgs::msg::Pose& pose,
                            rclcpp::Logger logger,
                            const std::string& title = "位姿信息");
};

} // namespace utils
} // namespace cr7_controller

#endif // POSE_UTILS_HPP_
