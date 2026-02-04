/**
 * @file cr7_robot_controller.hpp
 * @brief CR7机器人控制器头文件
 * 
 * 这个文件定义了CR7机器人控制器的核心类，包括：
 * 1. 机器人状态监控
 * 2. 运动规划执行
 * 3. 多路点控制
 * 4. 笛卡尔路径规划（通过模块集成）
 * 5. PILZ工业规划器（通过模块集成）
 * 6. 预设路径执行（通过模块集成）
 */

#ifndef CR7_ROBOT_CONTROLLER_HPP_
#define CR7_ROBOT_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "cr7_robot_controller/base/cr7_base_controller.hpp"
#include "cr7_robot_controller/cartesian/cr7_cartesian_planner.hpp"
#include "cr7_robot_controller/pilz/cr7_pilz_planner.hpp"
#include "cr7_robot_controller/path/cr7_path_executor.hpp"

namespace cr7_controller {

/**
 * @class CR7RobotController
 * @brief CR7机器人控制器主类
 * 
 * 这个类继承自CR7BaseController，提供：
 * 1. 基础运动控制（继承自CR7BaseController）
 * 2. 笛卡尔路径规划（通过CR7CartesianPlanner模块）
 * 3. PILZ工业规划器（通过CR7PilzPlanner模块）
 * 4. 预设路径执行（通过CR7PathExecutor模块）
 * 5. 机器人状态监控
 * 6. 错误处理
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
    
    /**
     * @brief 执行优化的笛卡尔路径（推荐使用）
     * @param waypoints 路径点序列
     * @return Result 规划结果
     */
    Result executeOptimizedCartesianPath(const std::vector<Waypoint>& waypoints);
    
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
    
    /**
     * @brief 设置PILZ规划器参数
     * @param planner_id 规划器ID
     * @return bool 设置成功返回true
     */
    bool setPilzPlanner(const std::string& planner_id);
    
    /**
     * @brief 获取可用的PILZ规划器列表
     * @return std::vector<std::string> 规划器列表
     */
    std::vector<std::string> getAvailablePilzPlanners() const;
    
    // ============================================================================
    // 预设路径执行方法（通过模块）
    // ============================================================================
    
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
    
private:
    // ============================================================================
    // 模块成员变量
    // ============================================================================
    
    std::shared_ptr<CR7CartesianPlanner> cartesian_planner_; ///< 笛卡尔路径规划器
    std::shared_ptr<CR7PilzPlanner> pilz_planner_;           ///< PILZ工业规划器
    std::shared_ptr<CR7PathExecutor> path_executor_;         ///< 预设路径执行器
};

} // namespace cr7_controller

#endif // CR7_ROBOT_CONTROLLER_HPP_