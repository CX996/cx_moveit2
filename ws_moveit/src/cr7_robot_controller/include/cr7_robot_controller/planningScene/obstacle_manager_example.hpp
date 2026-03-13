#ifndef CR7_OBSTACLE_MANAGER_EXAMPLE_HPP
#define CR7_OBSTACLE_MANAGER_EXAMPLE_HPP

#include <cr7_robot_controller/planningScene/obstacle_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cr7_robot_controller {

/**
 * @class ObstacleManagerExample
 * @brief 障碍物管理器使用示例
 * 
 * 该类展示了如何使用ObstacleManager来添加和删除障碍物
 */
class ObstacleManagerExample {
public:
    /**
     * @brief 构造函数
     * @param node ROS节点指针
     */
    ObstacleManagerExample(const rclcpp::Node::SharedPtr& node);

    /**
     * @brief 运行示例
     */
    void run();

private:
    rclcpp::Node::SharedPtr node_;  // ROS节点指针
    std::shared_ptr<ObstacleManager> obstacle_manager_;  // 障碍物管理器
};

} // namespace cr7_robot_controller

#endif // CR7_OBSTACLE_MANAGER_EXAMPLE_HPP
