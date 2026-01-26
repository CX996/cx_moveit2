/**
 * @file main.cpp
 * @brief CR7机器人控制主程序（带打印标识 + 参数化 + 注释）
 * 
 * 功能：
 * 1. 初始化ROS2
 * 2. 创建机器人控制器
 * 3. 打印启动信息
 * 4. 执行测试或焊接路径
 * 5. 等待用户退出
 * 6. 支持 launch 参数
 */

#include <iostream>
#include <memory>
#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include "cr7_robot_controller/cr7_robot_controller.hpp"

using namespace cr7_controller;

// 全局节点指针，用于信号处理
std::shared_ptr<rclcpp::Node> g_node = nullptr;

/**
 * @brief 信号处理函数
 * @param signal 信号编号
 * 
 * 用于 Ctrl+C 等中断，优雅退出 ROS2 节点
 */
void signalHandler(int signal) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "收到信号 " << signal << "，正在关闭..." << std::endl;

    if (g_node) {
        rclcpp::shutdown();
    }

    std::cout << "程序已关闭" << std::endl;
    std::cout << "========================================" << std::endl;
}

int main(int argc, char* argv[]) {
    // ==================== 初始化 ROS2 ====================
    rclcpp::init(argc, argv);

    // 创建 ROS2 节点
    auto node = std::make_shared<rclcpp::Node>("cr7_controller");
    g_node = node;  // 全局引用用于信号处理

    // ==================== 参数声明 ====================
    node->declare_parameter<bool>("test_mode", false);
    node->declare_parameter<bool>("debug_mode", false);
    // node->declare_parameter<bool>("use_sim_time", false);

    // 获取参数值
    bool test_mode = node->get_parameter("test_mode").as_bool();
    bool debug_mode = node->get_parameter("debug_mode").as_bool();
    bool use_sim_time = node->get_parameter("use_sim_time").as_bool();

    // ==================== 设置日志级别 ====================
    if (debug_mode) {
        auto ret = rcutils_logging_set_logger_level(
            node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        if (ret != RCUTILS_RET_OK) {
            std::cerr << "设置日志级别失败" << std::endl;
        }
    }

    // ==================== 信号处理 ====================
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // ==================== 打印启动信息 ====================
    std::cout << "========================================" << std::endl;
    std::cout << "启动 CR7 机器人控制器" << std::endl;
    std::cout << "模式: " << (test_mode ? "测试" : "焊接") << std::endl;
    std::cout << "调试: " << (debug_mode ? "启用" : "禁用") << std::endl;
    std::cout << "仿真时间: " << (use_sim_time ? "启用" : "禁用") << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        // ==================== 创建机器人控制器 ====================
        auto controller = std::make_shared<CR7RobotController>(node, "cr7_group");

        // 初始化控制器并等待 joint_states
        RCLCPP_INFO(node->get_logger(), "初始化机器人控制器...");
        if (!controller->initialize(10.0)) {
            RCLCPP_FATAL(node->get_logger(), "初始化失败");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "✓ 初始化成功");

        // 打印初始关节状态
        controller->printCurrentState();

        // ==================== 执行任务 ====================
        std::vector<CR7RobotController::Result> results;
        if (test_mode) {
            RCLCPP_INFO(node->get_logger(), "执行测试路径...");
            results = controller->executeTestPath();
        } else {
            RCLCPP_INFO(node->get_logger(), "执行焊接路径...");
            results = controller->executeWeldingPath();
        }

        // ==================== 打印任务结果 ====================
        int success_count = 0;
        for (const auto& r : results) {
            if (r == CR7RobotController::Result::SUCCESS) success_count++;
        }

        RCLCPP_INFO(node->get_logger(), "========================================");
        RCLCPP_INFO(node->get_logger(), "任务完成: 成功 %d/%zu", success_count, results.size());
        RCLCPP_INFO(node->get_logger(), "========================================");

        // 打印最终关节状态
        controller->printCurrentState();

        // ==================== 等待用户退出 ====================
        RCLCPP_INFO(node->get_logger(), "按 Ctrl+C 退出...");
        rclcpp::spin(node);

    } catch (const std::exception& e) {
        std::cerr << "========================================" << std::endl;
        std::cerr << "致命错误: " << e.what() << std::endl;
        std::cerr << "========================================" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // ==================== 程序结束 ====================
    rclcpp::shutdown();
    std::cout << "========================================" << std::endl;
    std::cout << "程序正常结束" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
