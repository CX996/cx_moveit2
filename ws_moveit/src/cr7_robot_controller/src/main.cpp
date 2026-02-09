/**
 * @file main.cpp
 * @brief CR7机器人控制主程序
 * 
 * 功能：
 * 1. 初始化ROS2
 * 2. 创建机器人控制器
 * 3. 执行指定的测试路径任务
 * 4. 参数化配置
 * 5. 轻量级设计，适合launch文件启动
 */

#include <iostream>
#include <memory>
#include <csignal>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "cr7_robot_controller/cr7_robot_controller.hpp"

using namespace cr7_controller;

// 全局变量
std::atomic<bool> g_running{true};
std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor = nullptr;

/**
 * @brief 信号处理函数
 * @param signal 信号编号
 * 
 * 用于 Ctrl+C 等中断，优雅退出 ROS2 节点
 */
void signalHandler(int signal)
{
    std::cout << "\n========================================" << std::endl;
    std::cout << "收到信号 " << signal << "，正在关闭..." << std::endl;

    g_running = false;
    
    // 停止执行器
    if (g_executor) {
        g_executor->cancel();
    }

    std::cout << "程序已关闭" << std::endl;
    std::cout << "========================================" << std::endl;
}

int main(int argc, char* argv[])
{
    // ==================== 初始化 ROS2 ====================
    rclcpp::init(argc, argv);

    // 创建 ROS2 节点
    auto node = std::make_shared<rclcpp::Node>("cr7_controller");

    // ==================== 参数声明 ====================
    node->declare_parameter<std::string>("execute_mode", "ompl_constraint_test"); // idle, test, cartesian_test, pilz_test, tool_axis_test, welding_test
    node->declare_parameter<double>("init_timeout", 10.0);

    // 获取参数值
    std::string execute_mode = node->get_parameter("execute_mode").as_string();
    double init_timeout = node->get_parameter("init_timeout").as_double();

    // ==================== 信号处理 ====================
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // ==================== 打印启动信息 ====================
    std::cout << "========================================" << std::endl;
    std::cout << "启动 CR7 机器人控制器" << std::endl;
    std::cout << "执行模式: " << execute_mode << std::endl;
    std::cout << "初始化超时: " << init_timeout << "秒" << std::endl;
    std::cout << "========================================" << std::endl;

    try
    {
        // ==================== 创建机器人控制器 ====================
        auto controller = std::make_shared<CR7RobotController>(node, "cr7_group");

        // ==================== 启动多线程执行器 ====================
        g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 4); // 使用4个线程

        g_executor->add_node(node);

        // 启动执行器线程
        std::thread executor_thread([this_executor = g_executor]() {
            this_executor->spin();
        });

        // 初始化控制器
        RCLCPP_INFO(node->get_logger(), "初始化机器人控制器...");
        if (!controller->initialize(init_timeout)) {
            RCLCPP_FATAL(node->get_logger(), "初始化失败");
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "✓ 初始化成功");

        // 打印初始状态
        controller->printCurrentState();

        // ==================== 执行指定模式 ====================
        if (execute_mode == "test")
        {
            RCLCPP_INFO(node->get_logger(), "执行基本测试路径...");
            controller->executeTestPath();
        }
        else if (execute_mode == "cartesian_test")
        {
            RCLCPP_INFO(node->get_logger(), "执行笛卡尔测试路径...");
            controller->executeCartesianTestPath();
        }
        else if (execute_mode == "pilz_test")
        {
            RCLCPP_INFO(node->get_logger(), "执行PILZ测试路径...");
            controller->executePilzTestPath();
        }
        else if (execute_mode == "tool_axis_test")
        {
            RCLCPP_INFO(node->get_logger(), "执行工具坐标系测试路径...");
            controller->executeToolAxisTestPath();
        }
        else if (execute_mode == "welding_test")
        {
            RCLCPP_INFO(node->get_logger(), "执行焊接路径测试...");
            controller->executeWeldingTestPath();
        }
        else if (execute_mode == "ompl_constraint_test")
        {
            RCLCPP_INFO(node->get_logger(), "执行OMPL约束规划测试...");
            
            auto result = controller->executeOMPLConstraintTest();
            RCLCPP_INFO(node->get_logger(), "OMPL约束规划测试结果: %s", 
                       controller->resultToString(result).c_str());
        }
        else if (execute_mode == "idle")
        {
            RCLCPP_INFO(node->get_logger(), "控制器处于空闲模式，等待外部调用...");
            // 空闲模式：不执行任何路径，等待外部服务调用
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "未知执行模式: %s，使用空闲模式", execute_mode.c_str());
        }

        // ==================== 等待执行器线程结束 ====================
        if (executor_thread.joinable())
        {
            executor_thread.join();
        }

    }
    catch (const std::exception& e)
    {
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
