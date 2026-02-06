/**
 * @file cr7_robot_controller.hpp
 * @brief CR7机器人控制器头文件
 * 
 * 这个文件定义了CR7机器人控制器的核心类，作为协调器整合所有模块。
 */

#ifndef CR7_ROBOT_CONTROLLER_HPP_
#define CR7_ROBOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "cr7_robot_controller/base/cr7_base_controller.hpp"
#include "cr7_robot_controller/planners/cartesian/cr7_cartesian_planner.hpp"
#include "cr7_robot_controller/planners/pilz/cr7_pilz_planner.hpp"
#include "cr7_robot_controller/planners/ompl/cr7_ompl_planner.hpp"
#include "cr7_robot_controller/path/cr7_path_executor.hpp"

namespace cr7_controller {

/**
 * @class CR7RobotController
 * @brief CR7机器人控制器主类
 * 
 * 这个类继承自CR7BaseController，作为协调器整合所有模块，提供：
 * 1. 基础运动控制（继承自CR7BaseController）
 * 2. 笛卡尔路径规划（通过CR7CartesianPlanner模块）
 * 3. PILZ工业规划器（通过CR7PilzPlanner模块）
 * 4. OMPL规划器（通过CR7OmplPlanner模块）
 * 5. 测试路径执行（通过CR7PathExecutor模块）
 * 6. 机器人状态监控
 * 7. 错误处理
 */
class CR7RobotController : public CR7BaseController {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param planning_group MoveIt规划组名称
     */
    CR7RobotController(rclcpp::Node::SharedPtr node, 
                      const std::string& planning_group = "cr7_group");
    
    // ============================================================================
    // 笛卡尔路径规划方法（通过模块）
    // ============================================================================
    
    /**
     * @brief 笛卡尔路径规划
     * @param waypoints 路径点序列
     * @param config 笛卡尔路径配置
     * @return Result 规划结果
     */
    Result executeCartesianPath(const std::vector<Waypoint>& waypoints,
                               const CartesianPathConfig& config);

    // ============================================================================
    // PILZ工业规划器方法（通过模块）
    // ============================================================================
    
    /**
     * @brief 测试PILZ规划器功能
     * @param planner_type 规划器类型
     * @return Result 测试结果
     */
    Result testPilzPlanner(PilzPlanner planner_type = PilzPlanner::LIN);
    
    /**
     * @brief 使用PILZ规划器执行直线运动
     * @param target_pose 目标位姿
     * @param config PILZ配置
     * @return Result 规划结果
     */
    Result moveWithPilzLin(const geometry_msgs::msg::Pose& target_pose,
                          const PilzConfig& config = PilzConfig());
    
    /**
     * @brief 使用PILZ规划器执行点对点运动
     * @param target_pose 目标位姿
     * @param config PILZ配置
     * @return Result 规划结果
     */
    Result moveWithPilzPtp(const geometry_msgs::msg::Pose& target_pose,
                          const PilzConfig& config = PilzConfig());
    
    /**
     * @brief 使用PILZ规划器执行圆周运动
     * @param intermediate_pose 中间点
     * @param target_pose 目标点
     * @param config PILZ配置
     * @return Result 规划结果
     */
    Result moveWithPilzCirc(const geometry_msgs::msg::Pose& intermediate_pose,
                           const geometry_msgs::msg::Pose& target_pose,
                           const PilzConfig& config = PilzConfig());
    
    // ============================================================================
    // 测试路径执行方法（通过模块）
    // ============================================================================
    
    /**
     * @brief 执行基本测试路径
     * @return std::vector<Result> 每个路点的结果
     */
    std::vector<Result> executeTestPath();
    
    /**
     * @brief 执行笛卡尔测试路径
     * @return Result 规划结果
     */
    Result executeCartesianTestPath();
    
    /**
     * @brief 执行PILZ测试路径
     * @return Result 规划结果
     */
    Result executePilzTestPath();
    
    /**
     * @brief 执行工具坐标系测试路径
     * @return Result 规划结果
     */
    Result executeToolAxisTestPath();
    
    /**
     * @brief 执行焊接路径测试
     * @return Result 规划结果
     */
    Result executeWeldingTestPath();
    
    // ============================================================================
    // 配置方法
    // ============================================================================
    
    /**
     * @brief 获取笛卡尔路径规划器
     * @return std::shared_ptr<CR7CartesianPlanner> 笛卡尔路径规划器指针
     */
    std::shared_ptr<CR7CartesianPlanner> getCartesianPlanner() 
    {
        return cartesian_planner_;
    }
    
    /**
     * @brief 获取PILZ规划器
     * @return std::shared_ptr<CR7PilzPlanner> PILZ规划器指针
     */
    std::shared_ptr<CR7PilzPlanner> getPilzPlanner() 
    {
        return pilz_planner_;
    }
    
    /**
     * @brief 获取路径执行器
     * @return std::shared_ptr<CR7PathExecutor> 路径执行器指针
     */
    std::shared_ptr<CR7PathExecutor> getPathExecutor() 
    {
        return path_executor_;
    }
    
    /**
     * @brief 获取OMPL规划器
     * @return std::shared_ptr<CR7OMPLPlanner> OMPL规划器指针
     */
    std::shared_ptr<CR7OMPLPlanner> getOMPLPlanner() 
    {
        return ompl_planner_;
    }
    
private:
    // ============================================================================
    // 模块成员变量
    // ============================================================================
    
    std::shared_ptr<CR7CartesianPlanner> cartesian_planner_; ///< 笛卡尔路径规划器
    std::shared_ptr<CR7PilzPlanner> pilz_planner_;           ///< PILZ工业规划器
    std::shared_ptr<CR7OMPLPlanner> ompl_planner_;           ///< OMPL规划器
    std::shared_ptr<CR7PathExecutor> path_executor_;         ///< 测试路径执行器
};

} // namespace cr7_controller

#endif // CR7_ROBOT_CONTROLLER_HPP_