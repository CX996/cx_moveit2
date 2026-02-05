/**
 * @file cr7_path_executor.hpp
 * @brief 预设路径执行模块头文件
 * 
 * 这个文件定义了CR7机器人的预设路径执行功能，包括：
 * 1. 焊接路径执行
 * 2. 测试路径执行
 * 3. 笛卡尔焊接路径执行
 * 4. PILZ焊接路径执行
 * 5. 工具坐标系路径执行
 */

#ifndef CR7_PATH_EXECUTOR_HPP_
#define CR7_PATH_EXECUTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"
#include "cr7_robot_controller/cartesian/cr7_cartesian_planner.hpp"
#include "cr7_robot_controller/pilz/cr7_pilz_planner.hpp"

namespace cr7_controller {

/**
 * @class CR7PathExecutor
 * @brief 预设路径执行器类
 * 
 * 这个类提供：
 * 1. 焊接路径执行
 * 2. 测试路径执行
 * 3. 笛卡尔焊接路径执行
 * 4. PILZ焊接路径执行
 * 5. 工具坐标系路径执行
 */
class CR7PathExecutor : public CR7BaseController {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param planning_group MoveIt规划组名称
     * @param cartesian_planner 笛卡尔路径规划器指针
     * @param pilz_planner PILZ规划器指针
     */
    CR7PathExecutor(
        rclcpp::Node::SharedPtr node,
        const std::string& planning_group,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
        std::shared_ptr<CR7CartesianPlanner> cartesian_planner,
        std::shared_ptr<CR7PilzPlanner> pilz_planner
    );
    
    /**
     * @brief 析构函数
     */
    ~CR7PathExecutor() = default;
    
    /**
     * @brief 执行预设焊接路径
     * @return std::vector<Result> 每个路点的结果
     */
    std::vector<Result> executeWeldingPath();
    
    /**
     * @brief 执行测试路径
     * @return std::vector<Result> 每个路点的结果
     */
    std::vector<Result> executeTestPath();
    
    /**
     * @brief 执行笛卡尔焊接路径
     * @return Result 规划结果
     */
    Result executeCartesianWeldingPath();
    
    /**
     * @brief 使用PILZ执行焊接点位路径
     * 先用关节空间移动到起点，再用PILZ执行到终点
     * @return Result 规划结果
     */
    Result executePilzWeldingPath();
    
    /**
     * @brief 使用工具坐标系进行点位执行
     * @return Result 规划结果
     */
    Result executeToolAxis();
    
private:
    /**
     * @brief 生成焊接路径点
     * @return std::vector<Waypoint> 焊接路径点
     */
    std::vector<Waypoint> generateWeldingPath();
    
    /**
     * @brief 生成测试路径点
     * @return std::vector<Waypoint> 测试路径点
     */
    std::vector<Waypoint> generateTestPath();
    
    /**
     * @brief 生成笛卡尔焊接路径点
     * @return std::vector<Waypoint> 笛卡尔路径点
     */
    std::vector<Waypoint> generateCartesianWeldingPath();
    
    /**
     * @brief 生成工具坐标系路径点
     * @return std::vector<Waypoint> 工具坐标系路径点
     */
    std::vector<Waypoint> generateToolAxisPath();
    
    // 成员变量
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<CR7CartesianPlanner> cartesian_planner_;
    std::shared_ptr<CR7PilzPlanner> pilz_planner_;
};

} // namespace cr7_controller

#endif // CR7_PATH_EXECUTOR_HPP_