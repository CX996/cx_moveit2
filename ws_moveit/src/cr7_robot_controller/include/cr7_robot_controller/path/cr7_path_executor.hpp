/**
 * @file cr7_path_executor.hpp
 * @brief 测试路径执行工具头文件
 * 
 * 这个文件定义了CR7机器人的测试路径执行功能，用于验证机器人的基本运动能力。
 */

#ifndef CR7_PATH_EXECUTOR_HPP_
#define CR7_PATH_EXECUTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "cr7_robot_controller/base/cr7_base_controller.hpp"
#include "cr7_robot_controller/planners/cartesian/cr7_cartesian_planner.hpp"
#include "cr7_robot_controller/planners/pilz/cr7_pilz_planner.hpp"
#include "cr7_robot_controller/planners/ompl/cr7_ompl_planner.hpp"

namespace cr7_controller {

/**
 * @class CR7PathExecutor
 * @brief 测试路径执行器类
 * 
 * 这个类提供各种测试路径的执行功能，用于验证机器人的运动能力。
 */
class CR7PathExecutor {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param move_group MoveGroup接口指针
     * @param cartesian_planner 笛卡尔路径规划器指针
     * @param pilz_planner PILZ规划器指针
     * @param ompl_planner OMPL规划器指针
     */
    CR7PathExecutor(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
        std::shared_ptr<CR7CartesianPlanner> cartesian_planner,
        std::shared_ptr<CR7PilzPlanner> pilz_planner,
        std::shared_ptr<CR7OMPLPlanner> ompl_planner
    );
    
    /**
     * @brief 析构函数
     */
    ~CR7PathExecutor() = default;
    
    /**
     * @brief 执行基本测试路径
     * @return std::vector<CR7BaseController::Result> 每个路点的结果
     */
    std::vector<CR7BaseController::Result> executeTestPath();
    
    /**
     * @brief 执行笛卡尔测试路径
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executeCartesianTestPath();
    
    /**
     * @brief 执行PILZ测试路径
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executePilzTestPath();
    
    /**
     * @brief 执行工具坐标系测试路径
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executeToolAxisTestPath();
    
    /**
     * @brief 执行焊接路径测试
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executeWeldingTestPath();
    
    /**
     * @brief 执行OMPL约束规划测试
     * @return CR7BaseController::Result 规划结果
     */
    CR7BaseController::Result executeOMPLConstraintTest();
    
private:
    // 成员变量
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<CR7CartesianPlanner> cartesian_planner_;
    std::shared_ptr<CR7PilzPlanner> pilz_planner_;
    std::shared_ptr<CR7OMPLPlanner> ompl_planner_;
    rclcpp::Logger logger_;
};

} // namespace cr7_controller

#endif // CR7_PATH_EXECUTOR_HPP_