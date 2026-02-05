// /**
//  * @file cr7_test_main.cpp
//  * @brief CR7机器人控制器测试主入口
//  * 
//  * 功能：
//  * 1. 提供命令行参数来选择要运行的测试
//  * 2. 支持自动化测试和手动测试两种模式
//  * 3. 集成所有测试用例
//  */

// #include <iostream>
// #include <memory>
// #include <csignal>
// #include <chrono>

// #include <rclcpp/rclcpp.hpp>
// #include "cr7_robot_controller/cr7_robot_controller.hpp"

// using namespace cr7_controller;

// // 全局变量
// std::atomic<bool> g_running{true};
// std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> g_executor = nullptr;

// /**
//  * @brief 信号处理函数
//  * @param signal 信号编号
//  */
// void signalHandler(int signal)
// {
//     std::cout << "\n========================================" << std::endl;
//     std::cout << "收到信号 " << signal << "，正在关闭..." << std::endl;

//     g_running = false;
    
//     // 停止执行器
//     if (g_executor) {
//         g_executor->cancel();
//     }

//     std::cout << "测试程序已关闭" << std::endl;
//     std::cout << "========================================" << std::endl;
// }

// /**
//  * @brief 执行单个路点测试
//  * @param controller 机器人控制器
//  */
// void testSingleWaypoint(std::shared_ptr<CR7RobotController> controller)
// {
//     std::cout << "\n--- 单个路点测试 ---" << std::endl;
    
//     // 获取当前位置作为参考
//     auto current_pose = controller->getCurrentPose();
//     std::cout << "当前位置: [" << current_pose.position.x << ", " 
//               << current_pose.position.y << ", " << current_pose.position.z << "]" << std::endl;
    
//     // 创建测试路点（当前位置偏移）
//     Waypoint test_wp("Test_Waypoint", 
//                      current_pose.position.x + 0.1,
//                      current_pose.position.y,
//                      current_pose.position.z,
//                      current_pose.orientation.x,
//                      current_pose.orientation.y,
//                      current_pose.orientation.z,
//                      current_pose.orientation.w);
    
//     std::cout << "目标位置: [" << test_wp.x << ", " << test_wp.y << ", " << test_wp.z << "]" << std::endl;
//     std::cout << "开始执行..." << std::endl;
    
//     auto result = controller->moveToWaypoint(test_wp);
//     std::cout << "执行结果: " << controller->resultToString(result) << std::endl;
// }

// /**
//  * @brief 自定义笛卡尔路径测试
//  * @param controller 机器人控制器
//  */
// void testCustomCartesianPath(std::shared_ptr<CR7RobotController> controller)
// {
//     std::cout << "\n--- 自定义笛卡尔路径测试 ---" << std::endl;
    
//     // 获取当前位置作为起始点
//     auto current_pose = controller->getCurrentPose();
    
//     // 创建自定义路径
//     std::vector<Waypoint> custom_path = {
//         Waypoint("Start", 
//                  current_pose.position.x,
//                  current_pose.position.y,
//                  current_pose.position.z,
//                  current_pose.orientation.x,
//                  current_pose.orientation.y,
//                  current_pose.orientation.z,
//                  current_pose.orientation.w),
        
//         Waypoint("Point1",
//                  current_pose.position.x + 0.1,
//                  current_pose.position.y,
//                  current_pose.position.z + 0.05,
//                  current_pose.orientation.x,
//                  current_pose.orientation.y,
//                  current_pose.orientation.z,
//                  current_pose.orientation.w),
        
//         Waypoint("Point2",
//                  current_pose.position.x + 0.1,
//                  current_pose.position.y + 0.1,
//                  current_pose.position.z + 0.1,
//                  current_pose.orientation.x,
//                  current_pose.orientation.y,
//                  current_pose.orientation.z,
//                  current_pose.orientation.w),
        
//         Waypoint("Point3",
//                  current_pose.position.x,
//                  current_pose.position.y + 0.1,
//                  current_pose.position.z + 0.05,
//                  current_pose.orientation.x,
//                  current_pose.orientation.y,
//                  current_pose.orientation.z,
//                  current_pose.orientation.w),
        
//         Waypoint("End",
//                  current_pose.position.x,
//                  current_pose.position.y,
//                  current_pose.position.z,
//                  current_pose.orientation.x,
//                  current_pose.orientation.y,
//                  current_pose.orientation.z,
//                  current_pose.orientation.w)
//     };
    
//     std::cout << "自定义路径包含 " << custom_path.size() << " 个路点" << std::endl;
//     std::cout << "开始执行笛卡尔路径规划..." << std::endl;
    
//     auto result = controller->executeCartesianPath(custom_path, controller->getCartesianConfig());
//     std::cout << "执行结果: " << controller->resultToString(result) << std::endl;
// }

// /**
//  * @brief 测试PILZ规划器
//  * @param controller 机器人控制器
//  * @param planner_type 规划器类型
//  */
// void testPilzPlanner(std::shared_ptr<CR7RobotController> controller, 
//                      PilzPlanner planner_type)
// {
//     std::cout << "\n开始测试PILZ " 
//               << CR7PilzPlanner::pilzPlannerToString(planner_type) 
//               << " 规划器..." << std::endl;
    
//     auto result = controller->testPilzPlanner(planner_type);
//     std::cout << "测试结果: " << controller->resultToString(result) << std::endl;
// }

// /**
//  * @brief 获取PILZ规划器列表
//  * @param controller 机器人控制器
//  */
// void showPilzPlanners(std::shared_ptr<CR7RobotController> controller)
// {
//     std::cout << "\n可用的PILZ规划器:" << std::endl;
//     auto planners = controller->getAvailablePilzPlanners();
    
//     if (planners.empty()) {
//         std::cout << "未找到可用的PILZ规划器" << std::endl;
//     } else {
//         for (size_t i = 0; i < planners.size(); ++i) {
//             std::cout << "  " << (i + 1) << ". " << planners[i] << std::endl;
//         }
//     }
// }

// /**
//  * @brief 运行所有测试
//  * @param controller 机器人控制器
//  */
// void runAllTests(std::shared_ptr<CR7RobotController> controller)
// {
//     std::cout << "========================================" << std::endl;
//     std::cout << "开始运行所有测试" << std::endl;
//     std::cout << "========================================" << std::endl;
    
//     // 1. 打印当前状态
//     std::cout << "\n1. 打印当前状态:" << std::endl;
//     controller->printCurrentState();
    
//     // 2. 单个路点测试
//     testSingleWaypoint(controller);
    
//     // 3. 测试路径
//     std::cout << "\n3. 执行测试路径:" << std::endl;
//     controller->executeTestPath();
    
//     // 4. 焊接路径
//     std::cout << "\n4. 执行焊接路径:" << std::endl;
//     controller->executeWeldingPath();
    
//     // 5. 笛卡尔焊接路径
//     std::cout << "\n5. 执行笛卡尔焊接路径:" << std::endl;
//     controller->executeCartesianWeldingPath();
    
//     // 6. 自定义笛卡尔路径
//     testCustomCartesianPath(controller);
    
//     // 7. PILZ规划器测试
//     std::cout << "\n7. PILZ规划器测试:" << std::endl;
//     showPilzPlanners(controller);
    
//     std::vector<CR7PilzPlanner::PilzPlanner> pilz_planners = {
//         CR7PilzPlanner::PilzPlanner::LIN,
//         CR7PilzPlanner::PilzPlanner::PTP,
//         CR7PilzPlanner::PilzPlanner::CIRC
//     };
    
//     for (const auto& planner : pilz_planners) 
//     {
//         testPilzPlanner(controller, planner);
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
    
//     // 8. PILZ焊接路径
//     std::cout << "\n8. 执行PILZ焊接路径:" << std::endl;
//     auto result = controller->executePilzWeldingPath();
//     std::cout << "执行结果: " << controller->resultToString(result) << std::endl;
    
//     std::cout << "========================================" << std::endl;
//     std::cout << "所有测试运行完成" << std::endl;
//     std::cout << "========================================" << std::endl;
// }

// int main(int argc, char* argv[])
// {
//     // ==================== 初始化 ROS2 ====================
//     rclcpp::init(argc, argv);

//     // 创建 ROS2 节点
//     auto node = std::make_shared<rclcpp::Node>("cr7_test_controller");

//     // ==================== 参数声明 ====================
//     node->declare_parameter<bool>("debug_mode", false);
//     node->declare_parameter<std::string>("test_type", "all"); // all, single_waypoint, cartesian, pilz

//     // 获取参数值
//     bool debug_mode = node->get_parameter("debug_mode").as_bool();
//     std::string test_type = node->get_parameter("test_type").as_string();

//     // ==================== 设置日志级别 ====================
//     if (debug_mode)
//     {
//         auto ret = rcutils_logging_set_logger_level(
//             node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
//         if (ret != RCUTILS_RET_OK)
//         {
//             std::cerr << "设置日志级别失败" << std::endl;
//         }
//     }

//     // ==================== 信号处理 ====================
//     signal(SIGINT, signalHandler);
//     signal(SIGTERM, signalHandler);

//     // ==================== 打印启动信息 ====================
//     std::cout << "========================================" << std::endl;
//     std::cout << "启动 CR7 机器人控制器测试" << std::endl;
//     std::cout << "测试类型: " << test_type << std::endl;
//     std::cout << "调试模式: " << (debug_mode ? "启用" : "禁用") << std::endl;
//     std::cout << "========================================" << std::endl;

//     try
//     {
//         // ==================== 创建机器人控制器 ====================
//         auto controller = std::make_shared<CR7RobotController>(node, "cr7_group");

//         // ==================== 启动多线程执行器 ====================
//         g_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
//             rclcpp::ExecutorOptions(), 8); // 使用8个线程

//         g_executor->add_node(node);

//         // 启动执行器线程
//         std::thread executor_thread([this_executor = g_executor]()
//         {
//             this_executor->spin();
//         });

//         // 初始化控制器
//         RCLCPP_INFO(node->get_logger(), "初始化机器人控制器...");
//         if (!controller->initialize(10.0))
//         {
//             RCLCPP_FATAL(node->get_logger(), "初始化失败");
//             rclcpp::shutdown();
//             if (executor_thread.joinable())
//             {
//                 executor_thread.join();
//             }
//             return 1;
//         }
//         RCLCPP_INFO(node->get_logger(), "✓ 初始化成功");

//         // 打印初始状态
//         controller->printCurrentState();

//         // ==================== 运行测试 ====================
//         if (test_type == "all")
//         {
//             runAllTests(controller);
//         }
//         else if (test_type == "single_waypoint")
//         {
//             testSingleWaypoint(controller);
//         }
//         else if (test_type == "cartesian")
//         {
//             testCustomCartesianPath(controller);
//         }
//         else if (test_type == "pilz")
//         {
//             showPilzPlanners(controller);
//             std::vector<CR7PilzPlanner::PilzPlanner> pilz_planners = 
//             {
//                 CR7PilzPlanner::PilzPlanner::LIN,
//                 CR7PilzPlanner::PilzPlanner::PTP,
//                 CR7PilzPlanner::PilzPlanner::CIRC
//             };
//             for (const auto& planner : pilz_planners)
//             {
//                 testPilzPlanner(controller, planner);
//                 std::this_thread::sleep_for(std::chrono::seconds(1));
//             }
//         }
//         else if (test_type == "test_path")
//         {
//             controller->executeTestPath();
//         }
//         else if (test_type == "welding_path")
//         {
//             controller->executeWeldingPath();
//         }
//         else if (test_type == "cartesian_welding")
//         {
//             controller->executeCartesianWeldingPath();
//         }
//         else if (test_type == "pilz_welding")
//         {
//             auto result = controller->executePilzWeldingPath();
//             std::cout << "执行结果: " << controller->resultToString(result) << std::endl;
//         }
//         else
//         {
//             std::cerr << "未知测试类型: " << test_type << std::endl;
//             std::cerr << "可用测试类型: all, single_waypoint, cartesian, pilz, test_path, welding_path, cartesian_welding, pilz_welding" << std::endl;
//             rclcpp::shutdown();
//             if (executor_thread.joinable())
//             {
//                 executor_thread.join();
//             }
//             return 1;
//         }

//         // ==================== 等待执行器线程结束 ====================
//         if (executor_thread.joinable())
//         {
//             executor_thread.join();
//         }

//     }
//     catch (const std::exception& e)
//     {
//         std::cerr << "========================================" << std::endl;
//         std::cerr << "测试过程中发生错误: " << e.what() << std::endl;
//         std::cerr << "========================================" << std::endl;
//         rclcpp::shutdown();
//         return 1;
//     }

//     // ==================== 程序结束 ====================
//     rclcpp::shutdown();
//     std::cout << "========================================" << std::endl;
//     std::cout << "测试程序正常结束" << std::endl;
//     std::cout << "========================================" << std::endl;

//     return 0;
// }