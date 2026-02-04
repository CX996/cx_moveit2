/**
 * @file cr7_controller_test.cpp
 * @brief CR7机器人控制器测试程序
 * 
 * 功能：
 * 1. 测试机器人控制器的各种功能
 * 2. 测试PILZ工业规划器
 * 3. 测试笛卡尔路径规划
 * 4. 测试预设路径执行
 */

#include <iostream>
#include <memory>
#include <csignal>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "cr7_robot_controller/cr7_robot_controller.hpp"

using namespace cr7_controller;

// 全局变量
std::atomic<bool> g_running{true};

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

    std::cout << "测试程序已关闭" << std::endl;
    std::cout << "========================================" << std::endl;
}

int main(int argc, char* argv[])
{
    // ==================== 初始化 ROS2 ====================
    rclcpp::init(argc, argv);

    // 创建 ROS2 节点
    auto node = std::make_shared<rclcpp::Node>("cr7_controller_test");

    // ==================== 参数声明 ====================
    node->declare_parameter<bool>("pilz_test", false);
    node->declare_parameter<std::string>("test_mode", "all"); // all, pilz, cartesian, path

    // 获取参数值
    bool pilz_test = node->get_parameter("pilz_test").as_bool();
    std::string test_mode = node->get_parameter("test_mode").as_string();

    // ==================== 信号处理 ====================
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // ==================== 打印启动信息 ====================
    std::cout << "========================================" << std::endl;
    std::cout << "启动 CR7 机器人控制器测试" << std::endl;
    std::cout << "测试模式: " << test_mode << std::endl;
    std::cout << "PILZ测试: " << (pilz_test ? "启用" : "禁用") << std::endl;
    std::cout << "========================================" << std::endl;

    try
    {
        // ==================== 创建机器人控制器 ====================
        auto controller = std::make_shared<CR7RobotController>(node, "cr7_group");

        // 初始化控制器
        RCLCPP_INFO(node->get_logger(), "初始化机器人控制器...");
        if (!controller->initialize(10.0))
        {
            RCLCPP_FATAL(node->get_logger(), "初始化失败");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "✓ 初始化成功");

        // 打印初始状态
        controller->printCurrentState();

        // ==================== 执行测试 ====================
        if (pilz_test || test_mode == "pilz")
        {
            // PILZ测试模式
            RCLCPP_INFO(node->get_logger(), "执行PILZ规划器测试...");
            
            // 测试所有PILZ规划器
            std::vector<CR7PilzPlanner::PilzPlanner> planners = 
            {
                CR7PilzPlanner::PilzPlanner::LIN,
                CR7PilzPlanner::PilzPlanner::PTP,
                CR7PilzPlanner::PilzPlanner::CIRC
            };
            
            for (const auto& planner : planners)
            {
                std::cout << "\n测试PILZ " 
                          << CR7PilzPlanner::pilzPlannerToString(planner) 
                          << " 规划器..." << std::endl;
                auto result = controller->testPilzPlanner(planner);
                std::cout << "测试结果: " << controller->resultToString(result) << std::endl;
                
                if (!g_running)
                {
                    break;
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            
        }
        else if (test_mode == "cartesian")
        {
            RCLCPP_INFO(node->get_logger(), "执行笛卡尔路径规划测试...");
            controller->executeCartesianWeldingPath();
        }
        else if (test_mode == "path")
        {
            RCLCPP_INFO(node->get_logger(), "执行预设路径测试...");
            controller->executeTestPath();
        }
        else if (test_mode == "all")
        {
            // 执行所有测试
            RCLCPP_INFO(node->get_logger(), "执行完整测试套件...");
            
            // 1. 测试预设路径
            RCLCPP_INFO(node->get_logger(), "\n1. 执行测试路径...");
            controller->executeTestPath();
            
            if (!g_running)
            {
                rclcpp::shutdown();
                return 0;
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // 2. 测试笛卡尔路径
            RCLCPP_INFO(node->get_logger(), "\n2. 执行笛卡尔焊接路径...");
            controller->executeCartesianWeldingPath();
            
            if (!g_running)
            {
                rclcpp::shutdown();
                return 0;
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            // 3. 测试PILZ路径
            RCLCPP_INFO(node->get_logger(), "\n3. 执行PILZ焊接路径...");
            controller->executePilzWeldingPath();
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "未知测试模式: %s，使用默认测试", test_mode.c_str());
            controller->executeTestPath();
        }

    }
    catch (const std::exception& e)
    {
        std::cerr << "========================================" << std::endl;
        std::cerr << "测试过程中发生错误: " << e.what() << std::endl;
        std::cerr << "========================================" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // ==================== 程序结束 ====================
    rclcpp::shutdown();
    std::cout << "========================================" << std::endl;
    std::cout << "测试程序正常结束" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}