/**
 * @file main.cpp
 * @brief CR7机器人控制主程序（交互式测试框架）
 * 
 * 功能：
 * 1. 初始化ROS2
 * 2. 创建机器人控制器
 * 3. 交互式命令行界面
 * 4. 支持多种测试模式
 * 5. 参数化配置
 */

#include <iostream>
#include <memory>
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "cr7_robot_controller/cr7_robot_controller.hpp"

using namespace cr7_controller;

// 全局变量
std::shared_ptr<rclcpp::Node> g_node = nullptr;
std::atomic<bool> g_running{true};
std::atomic<bool> g_processing_command{false};

/**
 * @brief 设置终端为非阻塞模式
 */
void setNonBlockingInput() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~(ICANON | ECHO); // 非规范模式，无回显
    ttystate.c_cc[VMIN] = 1;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

/**
 * @brief 恢复终端设置
 */
void resetTerminal() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

/**
 * @brief 非阻塞读取字符
 * @return int 读取的字符，如果没有输入返回-1
 */
int getCharNonBlocking() {
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_SET(STDIN_FILENO, &fds);
    
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) == 1) {
        return getchar();
    }
    return -1;
}

/**
 * @brief 信号处理函数
 * @param signal 信号编号
 * 
 * 用于 Ctrl+C 等中断，优雅退出 ROS2 节点
 */
void signalHandler(int signal) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "收到信号 " << signal << "，正在关闭..." << std::endl;

    g_running = false;
    resetTerminal(); // 恢复终端设置
    
    if (g_node) {
        rclcpp::shutdown();
    }

    std::cout << "程序已关闭" << std::endl;
    std::cout << "========================================" << std::endl;
}

/**
 * @brief 显示主菜单
 */
void showMenu() {
    std::cout << "\n========================================" << std::endl;
    std::cout << "CR7 机器人控制器 - 测试菜单" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "1. 打印当前状态" << std::endl;
    std::cout << "2. 执行单个路点测试" << std::endl;
    std::cout << "3. 执行测试路径（点对点）" << std::endl;
    std::cout << "4. 执行焊接路径（点对点）" << std::endl;
    std::cout << "5. 执行笛卡尔焊接路径" << std::endl;
    std::cout << "6. 自定义笛卡尔路径" << std::endl;
    std::cout << "7. 设置规划参数" << std::endl;
    std::cout << "8. 设置笛卡尔参数" << std::endl;
    std::cout << "0. 退出程序" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "请选择操作 (0-8): ";
    std::cout.flush(); // 确保输出被刷新
}

/**
 * @brief 结果转字符串（辅助函数）
 * @param result 规划结果
 * @return std::string 结果描述
 */
std::string resultToString(CR7RobotController::Result result) {
    switch (result) {
        case CR7RobotController::Result::SUCCESS: return "成功";
        case CR7RobotController::Result::PLANNING_FAILED: return "规划失败";
        case CR7RobotController::Result::EXECUTION_FAILED: return "执行失败";
        case CR7RobotController::Result::INVALID_INPUT: return "输入无效";
        case CR7RobotController::Result::ROBOT_NOT_READY: return "机器人未就绪";
        case CR7RobotController::Result::TIMEOUT: return "超时";
        case CR7RobotController::Result::CARTESIAN_FAILED: return "笛卡尔规划失败";
        default: return "未知结果";
    }
}

/**
 * @brief 执行单个路点测试
 * @param controller 机器人控制器
 */
void testSingleWaypoint(std::shared_ptr<CR7RobotController> controller) {
    std::cout << "\n--- 单个路点测试 ---" << std::endl;
    
    // 获取当前位置作为参考
    auto current_pose = controller->getCurrentPose();
    std::cout << "当前位置: [" << current_pose.position.x << ", " 
              << current_pose.position.y << ", " << current_pose.position.z << "]" << std::endl;
    
    // 创建测试路点（当前位置偏移）
    Waypoint test_wp("Test_Waypoint", 
                     current_pose.position.x + 0.1,
                     current_pose.position.y,
                     current_pose.position.z,
                     current_pose.orientation.x,
                     current_pose.orientation.y,
                     current_pose.orientation.z,
                     current_pose.orientation.w);
    
    std::cout << "目标位置: [" << test_wp.x << ", " << test_wp.y << ", " << test_wp.z << "]" << std::endl;
    std::cout << "开始执行..." << std::endl;
    
    auto result = controller->moveToWaypoint(test_wp);
    std::cout << "执行结果: " << resultToString(result) << std::endl;
}

/**
 * @brief 设置规划参数
 * @param controller 机器人控制器
 */
void setPlanningParameters(std::shared_ptr<CR7RobotController> controller) {
    std::cout << "\n--- 设置规划参数 ---" << std::endl;
    
    double planning_time, velocity_factor, acceleration_factor;
    
    std::cout << "规划时间 (秒): ";
    std::cin >> planning_time;
    std::cout << "速度因子 (0.0-1.0): ";
    std::cin >> velocity_factor;
    std::cout << "加速度因子 (0.0-1.0): ";
    std::cin >> acceleration_factor;
    
    // 清除输入缓冲区
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    controller->setPlanningTime(planning_time);
    controller->setVelocityFactor(velocity_factor);
    controller->setAccelerationFactor(acceleration_factor);
    
    std::cout << "参数设置完成" << std::endl;
}

/**
 * @brief 设置笛卡尔参数
 * @param controller 机器人控制器
 */
void setCartesianParameters(std::shared_ptr<CR7RobotController> controller) {
    std::cout << "\n--- 设置笛卡尔参数 ---" << std::endl;
    
    auto& config = controller->getCartesianConfig();
    double max_step, jump_threshold, eef_step;
    int avoid_collisions;
    
    std::cout << "最大步长 (米): ";
    std::cin >> max_step;
    std::cout << "跳跃阈值: ";
    std::cin >> jump_threshold;
    std::cout << "末端执行器步长 (米): ";
    std::cin >> eef_step;
    std::cout << "避障 (1=是, 0=否): ";
    std::cin >> avoid_collisions;
    
    // 清除输入缓冲区
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    
    config.max_step = max_step;
    config.jump_threshold = jump_threshold;
    config.eef_step = eef_step;
    config.avoid_collisions = (avoid_collisions != 0);
    
    std::cout << "笛卡尔参数设置完成" << std::endl;
}

/**
 * @brief 自定义笛卡尔路径测试
 * @param controller 机器人控制器
 */
void testCustomCartesianPath(std::shared_ptr<CR7RobotController> controller) {
    std::cout << "\n--- 自定义笛卡尔路径测试 ---" << std::endl;
    
    // 获取当前位置作为起始点
    auto current_pose = controller->getCurrentPose();
    
    // 创建自定义路径
    std::vector<Waypoint> custom_path = {
        Waypoint("Start", 
                 current_pose.position.x,
                 current_pose.position.y,
                 current_pose.position.z,
                 current_pose.orientation.x,
                 current_pose.orientation.y,
                 current_pose.orientation.z,
                 current_pose.orientation.w),
        
        Waypoint("Point1",
                 current_pose.position.x + 0.1,
                 current_pose.position.y,
                 current_pose.position.z + 0.05,
                 current_pose.orientation.x,
                 current_pose.orientation.y,
                 current_pose.orientation.z,
                 current_pose.orientation.w),
        
        Waypoint("Point2",
                 current_pose.position.x + 0.1,
                 current_pose.position.y + 0.1,
                 current_pose.position.z + 0.1,
                 current_pose.orientation.x,
                 current_pose.orientation.y,
                 current_pose.orientation.z,
                 current_pose.orientation.w),
        
        Waypoint("Point3",
                 current_pose.position.x,
                 current_pose.position.y + 0.1,
                 current_pose.position.z + 0.05,
                 current_pose.orientation.x,
                 current_pose.orientation.y,
                 current_pose.orientation.z,
                 current_pose.orientation.w),
        
        Waypoint("End",
                 current_pose.position.x,
                 current_pose.position.y,
                 current_pose.position.z,
                 current_pose.orientation.x,
                 current_pose.orientation.y,
                 current_pose.orientation.z,
                 current_pose.orientation.w)
    };
    
    std::cout << "自定义路径包含 " << custom_path.size() << " 个路点" << std::endl;
    std::cout << "开始执行笛卡尔路径规划..." << std::endl;
    
    auto result = controller->executeCartesianPath(custom_path, controller->getCartesianConfig());
    std::cout << "执行结果: " << resultToString(result) << std::endl;
}

/**
 * @brief 处理用户输入命令
 * @param controller 机器人控制器
 * @param choice 用户选择
 */
void processCommand(std::shared_ptr<CR7RobotController> controller, char choice) {
    g_processing_command = true;
    
    switch (choice) {
        case '0': // 退出
            g_running = false;
            std::cout << "\n退出程序..." << std::endl;
            break;
            
        case '1': // 打印状态
            std::cout << std::endl;
            controller->printCurrentState();
            break;
            
        case '2': // 单个路点测试
            std::cout << std::endl;
            testSingleWaypoint(controller);
            break;
            
        case '3': // 测试路径
            std::cout << "\n执行测试路径..." << std::endl;
            controller->executeTestPath();
            break;
            
        case '4': // 焊接路径
            std::cout << "\n执行焊接路径..." << std::endl;
            controller->executeWeldingPath();
            break;
            
        case '5': // 笛卡尔焊接路径
            std::cout << "\n执行笛卡尔焊接路径..." << std::endl;
            controller->executeCartesianWeldingPath();
            break;
            
        case '6': // 自定义笛卡尔路径
            std::cout << std::endl;
            testCustomCartesianPath(controller);
            break;
            
        case '7': // 设置规划参数
            std::cout << std::endl;
            setPlanningParameters(controller);
            break;
            
        case '8': // 设置笛卡尔参数
            std::cout << std::endl;
            setCartesianParameters(controller);
            break;
            
        default:
            std::cout << "\n无效选择，请重新输入 (0-8)" << std::endl;
            break;
    }
    
    g_processing_command = false;
}

/**
 * @brief ROS2 自旋线程函数
 * @param node ROS2 节点
 */
void spinThread(std::shared_ptr<rclcpp::Node> node) {
    rclcpp::spin(node);
}

/**
 * @brief 用户输入处理线程
 * @param controller 机器人控制器
 */
void inputThread(std::shared_ptr<CR7RobotController> controller) {
    // 设置非阻塞输入
    setNonBlockingInput();
    
    bool menu_shown = false;
    
    while (g_running && rclcpp::ok()) {
        if (!menu_shown && !g_processing_command) {
            showMenu();
            menu_shown = true;
        }
        
        // 非阻塞读取输入
        int ch = getCharNonBlocking();
        
        if (ch != -1 && !g_processing_command) {
            processCommand(controller, static_cast<char>(ch));
            menu_shown = false;
        }
        
        // 短暂休眠，避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 恢复终端设置
    resetTerminal();
}

int main(int argc, char* argv[]) {
    // ==================== 初始化 ROS2 ====================
    rclcpp::init(argc, argv);

    // 创建 ROS2 节点
    auto node = std::make_shared<rclcpp::Node>("cr7_controller");
    g_node = node;

    // ==================== 参数声明 ====================
    node->declare_parameter<bool>("test_mode", false);
    node->declare_parameter<bool>("debug_mode", false);
    node->declare_parameter<bool>("interactive_mode", false);

    // 获取参数值
    bool test_mode = node->get_parameter("test_mode").as_bool();
    bool debug_mode = node->get_parameter("debug_mode").as_bool();
    bool interactive_mode = node->get_parameter("interactive_mode").as_bool();

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
    std::cout << "交互模式: " << (interactive_mode ? "启用" : "禁用") << std::endl;
    std::cout << "调试模式: " << (debug_mode ? "启用" : "禁用") << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        // ==================== 创建机器人控制器 ====================
        auto controller = std::make_shared<CR7RobotController>(node, "cr7_group");

        // 初始化控制器
        RCLCPP_INFO(node->get_logger(), "初始化机器人控制器...");
        if (!controller->initialize(10.0)) {
            RCLCPP_FATAL(node->get_logger(), "初始化失败");
            resetTerminal(); // 确保恢复终端
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "✓ 初始化成功");

        // 打印初始状态
        controller->printCurrentState();

        // ==================== 启动ROS自旋线程 ====================
        std::thread ros_thread(spinThread, node);

        // ==================== 交互式测试循环 ====================
        if (interactive_mode) {
            std::cout << "\n进入交互模式，可以使用菜单控制机器人..." << std::endl;
            
            // 启动用户输入线程
            std::thread input_thread(inputThread, controller);
            
            // 等待程序结束
            while (g_running && rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            
            // 等待线程结束
            if (input_thread.joinable()) {
                input_thread.join();
            }
        } else {
            // ==================== 非交互模式 ====================
            if (test_mode) {
                RCLCPP_INFO(node->get_logger(), "执行测试路径...");
                controller->executeTestPath();
            } else {
                RCLCPP_INFO(node->get_logger(), "执行焊接路径...");
                controller->executeCartesianWeldingPath();
            }
            
            // 等待用户退出
            std::cout << "按 Ctrl+C 退出..." << std::endl;
            while (g_running && rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        // ==================== 等待ROS线程结束 ====================
        if (ros_thread.joinable()) {
            ros_thread.join();
        }

    } catch (const std::exception& e) {
        std::cerr << "========================================" << std::endl;
        std::cerr << "致命错误: " << e.what() << std::endl;
        std::cerr << "========================================" << std::endl;
        resetTerminal();
        rclcpp::shutdown();
        return 1;
    }

    // ==================== 程序结束 ====================
    resetTerminal(); // 确保恢复终端设置
    rclcpp::shutdown();
    std::cout << "========================================" << std::endl;
    std::cout << "程序正常结束" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}