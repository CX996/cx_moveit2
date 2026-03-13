#include <cr7_robot_controller/planningScene/obstacle_manager_example.hpp>
#include <chrono>
#include <thread>

namespace cr7_robot_controller {

ObstacleManagerExample::ObstacleManagerExample(const rclcpp::Node::SharedPtr& node) 
    : node_(node) {
    // 创建障碍物管理器，使用默认配置，不自动添加障碍物
    obstacle_manager_ = std::make_shared<ObstacleManager>(node_, "dummy_link", false, ObstacleManager::ConfigType::DEFAULT);
}

void ObstacleManagerExample::run() {
    RCLCPP_INFO(node_->get_logger(), "=== 障碍物管理器使用示例 ===");
    
    // 1. 清理场景
    RCLCPP_INFO(node_->get_logger(), "1. 清理场景中的所有障碍物...");
    obstacle_manager_->cleanupScene();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 2. 添加默认障碍物
    RCLCPP_INFO(node_->get_logger(), "2. 添加默认障碍物（基础配置）...");
    obstacle_manager_->addCollisionObjects();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 3. 显示当前障碍物
    RCLCPP_INFO(node_->get_logger(), "3. 当前配置中的障碍物ID:");
    auto current_ids = obstacle_manager_->getCurrentObstacleIds();
    for (const auto& id : current_ids) {
        RCLCPP_INFO(node_->get_logger(), "   - %s", id.c_str());
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 4. 切换到扩展配置并添加
    RCLCPP_INFO(node_->get_logger(), "4. 切换到扩展配置并添加障碍物...");
    obstacle_manager_->setConfigType(ObstacleManager::ConfigType::EXTENDED);
    obstacle_manager_->addCollisionObjects();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 5. 显示所有已知障碍物ID
    RCLCPP_INFO(node_->get_logger(), "5. 所有已知障碍物ID:");
    auto all_ids = obstacle_manager_->getAllObstacleIds();
    for (const auto& id : all_ids) {
        RCLCPP_INFO(node_->get_logger(), "   - %s", id.c_str());
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 6. 删除基础障碍物
    RCLCPP_INFO(node_->get_logger(), "6. 删除基础障碍物（box_1, box_2, box_3）...");
    obstacle_manager_->removeBaseObstacles();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 7. 重置为默认配置并添加
    RCLCPP_INFO(node_->get_logger(), "7. 重置为默认配置并添加障碍物...");
    obstacle_manager_->resetToDefault();
    obstacle_manager_->addCollisionObjects();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 8. 清理所有障碍物
    RCLCPP_INFO(node_->get_logger(), "8. 清理所有障碍物...");
    obstacle_manager_->removeAllObstacles();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // 9. 创建自定义障碍物
    RCLCPP_INFO(node_->get_logger(), "9. 创建并添加自定义障碍物...");
    std::vector<ObstacleManager::ObstacleConfig> custom_obstacles;
    
    // 创建一个自定义立方体
    ObstacleManager::ObstacleConfig custom_box;
    custom_box.id = "custom_box";
    custom_box.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    custom_box.dimensions = {0.2, 0.2, 0.2};
    custom_box.pose.position.x = 0.5;
    custom_box.pose.position.y = 0.0;
    custom_box.pose.position.z = 0.1;
    custom_box.pose.orientation.w = 1.0;
    custom_obstacles.push_back(custom_box);
    
    // 添加自定义障碍物
    obstacle_manager_->addCollisionObjects(custom_obstacles);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // 10. 最终清理
    RCLCPP_INFO(node_->get_logger(), "10. 最终清理所有障碍物...");
    obstacle_manager_->cleanupScene();
    
    RCLCPP_INFO(node_->get_logger(), "=== 障碍物管理器使用示例完成 ===");
}

} // namespace cr7_robot_controller
