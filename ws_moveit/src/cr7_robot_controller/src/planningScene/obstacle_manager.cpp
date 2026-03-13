#include <cr7_robot_controller/planningScene/obstacle_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace cr7_robot_controller {

ObstacleManager::ObstacleManager(const rclcpp::Node::SharedPtr& node, 
                               const std::string& world_frame, 
                               bool auto_add, 
                               ConfigType config_type) 
    : node_(node), 
      world_frame_(world_frame), 
      planning_scene_interface_() {
    
    // 初始化障碍物配置
    initializeObstacleConfigs();
    
    // 设置配置类型
    setConfigType(config_type);
    
    // 如果需要自动添加障碍物，则启动定时器
    if (auto_add) {
        RCLCPP_INFO(node_->get_logger(), "将在2秒后自动添加 %zu 个障碍物...", obstacles_config_.size());
        timer_ = node_->create_wall_timer(
            std::chrono::seconds(2), 
            [this]() 
            { 
                this->addCollisionObjects(); 
                this->timer_->cancel(); 
            }
        );
    } else {
        RCLCPP_INFO(node_->get_logger(), "自动添加已禁用，请手动调用addCollisionObjects()");
    }
}

ObstacleManager::~ObstacleManager() {
    // 清理资源
    if (timer_ && timer_->is_ready()) {
        timer_->cancel();
    }
}

void ObstacleManager::initializeObstacleConfigs() {
    // 基础障碍物配置
    ObstacleConfig box1;
    box1.id = "box_1";
    box1.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    box1.dimensions = {0.8, 0.01, 0.5};
    box1.pose.position.x = 0.4;
    box1.pose.position.y = 0.45;
    box1.pose.position.z = 0.25;
    box1.pose.orientation.w = 1.0;
    base_obstacles_.push_back(box1);
    
    ObstacleConfig box2;
    box2.id = "box_2";
    box2.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    box2.dimensions = {0.8, 0.01, 0.5};
    box2.pose.position.x = 0.4;
    box2.pose.position.y = -0.45;
    box2.pose.position.z = 0.25;
    box2.pose.orientation.w = 1.0;
    base_obstacles_.push_back(box2);
    
    ObstacleConfig box3;
    box3.id = "box_3";
    box3.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    box3.dimensions = {0.01, 0.9, 0.5};
    box3.pose.position.x = 0.8;
    box3.pose.position.y = 0.0;
    box3.pose.position.z = 0.25;
    box3.pose.orientation.w = 1.0;
    base_obstacles_.push_back(box3);
    
    ObstacleConfig box4;
    box4.id = "box_4";
    box4.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    box4.dimensions = {1.0, 0.9, 0.01};
    box4.pose.position.x = 0.3;
    box4.pose.position.y = 0.0;
    box4.pose.position.z = -0.005;
    box4.pose.orientation.w = 1.0;
    base_obstacles_.push_back(box4);
    
    // 扩展障碍物配置
    extended_obstacles_ = base_obstacles_;
    
    ObstacleConfig box5;
    box5.id = "box_5";
    box5.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    box5.dimensions = {0.5, 0.01, 0.3};
    box5.pose.position.x = 0.1;
    box5.pose.position.y = 0.6;
    box5.pose.position.z = 0.15;
    box5.pose.orientation.w = 1.0;
    extended_obstacles_.push_back(box5);
    
    ObstacleConfig box6;
    box6.id = "box_6";
    box6.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    box6.dimensions = {0.5, 0.01, 0.3};
    box6.pose.position.x = 0.1;
    box6.pose.position.y = -0.6;
    box6.pose.position.z = 0.15;
    box6.pose.orientation.w = 1.0;
    extended_obstacles_.push_back(box6);
    
    ObstacleConfig box7;
    box7.id = "box_7";
    box7.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    box7.dimensions = {0.01, 0.5, 0.3};
    box7.pose.position.x = 0.9;
    box7.pose.position.y = 0.0;
    box7.pose.position.z = 0.15;
    box7.pose.orientation.w = 1.0;
    extended_obstacles_.push_back(box7);
    
    // 自定义测试障碍物配置
    ObstacleConfig cylinder1;
    cylinder1.id = "test_cylinder_1";
    cylinder1.type.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder1.dimensions = {0.1, 0.05}; // 高度, 半径
    cylinder1.pose.position.x = 0.4;
    cylinder1.pose.position.y = 0.0;
    cylinder1.pose.position.z = 0.05;
    cylinder1.pose.orientation.w = 1.0;
    custom_test_obstacles_.push_back(cylinder1);
    
    ObstacleConfig sphere1;
    sphere1.id = "test_sphere_1";
    sphere1.type.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere1.dimensions = {0.08}; // 半径
    sphere1.pose.position.x = 0.5;
    sphere1.pose.position.y = 0.3;
    sphere1.pose.position.z = 0.08;
    sphere1.pose.orientation.w = 1.0;
    custom_test_obstacles_.push_back(sphere1);
    
    ObstacleConfig small_box;
    small_box.id = "test_box_small";
    small_box.type.type = shape_msgs::msg::SolidPrimitive::BOX;
    small_box.dimensions = {0.1, 0.1, 0.1};
    small_box.pose.position.x = 0.5;
    small_box.pose.position.y = -0.3;
    small_box.pose.position.z = 0.05;
    small_box.pose.orientation.w = 1.0;
    custom_test_obstacles_.push_back(small_box);
}

bool ObstacleManager::addCollisionObjects(const std::vector<ObstacleConfig>& config_override, bool force) {
    try {
        // 确定使用哪个配置
        const std::vector<ObstacleConfig>& current_config = config_override.empty() ? obstacles_config_ : config_override;
        
        if (config_override.empty()) {
            RCLCPP_INFO(node_->get_logger(), "使用当前配置，添加 %zu 个障碍物", current_config.size());
        } else {
            RCLCPP_INFO(node_->get_logger(), "使用覆盖配置，添加 %zu 个障碍物", current_config.size());
        }
        
        if (current_config.empty()) {
            RCLCPP_WARN(node_->get_logger(), "配置为空，不添加任何障碍物");
            return true;
        }
        
        // 创建碰撞物体列表
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        
        // 遍历所有障碍物配置，创建碰撞物体
        for (const auto& cfg : current_config) {
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.id = cfg.id;
            collision_object.header.frame_id = world_frame_;
            collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
            
            // 添加几何体
            collision_object.primitives.push_back(cfg.type);
            collision_object.primitive_poses.push_back(cfg.pose);
            
            collision_objects.push_back(collision_object);
        }
        
        if (collision_objects.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "没有有效的障碍物可以添加");
            return false;
        }
        
        // 添加碰撞物体到规划场景
        planning_scene_interface_.addCollisionObjects(collision_objects);
        
        RCLCPP_INFO(node_->get_logger(), "✅ 成功添加 %zu 个障碍物", collision_objects.size());
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "添加障碍物异常: %s", e.what());
        return false;
    }
}

bool ObstacleManager::removeCollisionObject(const std::string& object_id, bool silent) {
    try {
        // 删除障碍物
        std::vector<std::string> object_ids = {object_id};
        planning_scene_interface_.removeCollisionObjects(object_ids);
        
        if (!silent) {
            RCLCPP_INFO(node_->get_logger(), "✅ 成功删除障碍物 '%s'", object_id.c_str());
        }
        return true;
        
    } catch (const std::exception& e) {
        if (!silent) {
            RCLCPP_ERROR(node_->get_logger(), "删除障碍物异常: %s", e.what());
        }
        return false;
    }
}

bool ObstacleManager::removeMultipleCollisionObjects(const std::vector<std::string>& object_ids, bool silent) {
    bool success = true;
    
    for (const auto& obj_id : object_ids) {
        if (!removeCollisionObject(obj_id, silent)) {
            success = false;
        }
    }
    
    return success;
}

bool ObstacleManager::removeObstaclesByIds(const std::vector<std::string>& object_ids, bool silent) {
    if (object_ids.empty()) {
        if (!silent) {
            RCLCPP_INFO(node_->get_logger(), "没有指定要删除的障碍物");
        }
        return true;
    }
    
    if (!silent) {
        RCLCPP_INFO(node_->get_logger(), "正在删除 %zu 个指定的障碍物", object_ids.size());
    }
    return removeMultipleCollisionObjects(object_ids, silent);
}

bool ObstacleManager::cleanupScene(const std::vector<std::string>* obstacle_ids) {
    std::vector<std::string> all_known_ids;
    
    if (obstacle_ids == nullptr) {
        // 尝试清理所有已知障碍物
        for (const auto& obstacle : base_obstacles_) {
            all_known_ids.push_back(obstacle.id);
        }
        for (const auto& obstacle : extended_obstacles_) {
            if (std::find(all_known_ids.begin(), all_known_ids.end(), obstacle.id) == all_known_ids.end()) {
                all_known_ids.push_back(obstacle.id);
            }
        }
        for (const auto& obstacle : custom_test_obstacles_) {
            if (std::find(all_known_ids.begin(), all_known_ids.end(), obstacle.id) == all_known_ids.end()) {
                all_known_ids.push_back(obstacle.id);
            }
        }
    } else {
        all_known_ids = *obstacle_ids;
    }
    
    RCLCPP_INFO(node_->get_logger(), "清理场景中的障碍物");
    
    // 静默模式删除，不记录错误
    removeMultipleCollisionObjects(all_known_ids, true);
    
    // 等待一下确保删除完成
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    RCLCPP_INFO(node_->get_logger(), "场景清理完成");
    return true;
}

bool ObstacleManager::removeAllConfiguredObstacles() {
    // 从配置中提取所有障碍物ID
    std::vector<std::string> object_ids;
    for (const auto& cfg : obstacles_config_) {
        object_ids.push_back(cfg.id);
    }
    
    if (object_ids.empty()) {
        RCLCPP_INFO(node_->get_logger(), "没有配置的障碍物需要删除");
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), "正在删除 %zu 个配置的障碍物", object_ids.size());
    return removeMultipleCollisionObjects(object_ids);
}

bool ObstacleManager::removeBaseObstacles() {
    std::vector<std::string> base_ids = {"box_1", "box_2", "box_3"};
    
    RCLCPP_INFO(node_->get_logger(), "正在删除基础障碍物");
    return removeMultipleCollisionObjects(base_ids);
}

bool ObstacleManager::removeAllObstacles() {
    // 收集所有可能的ID
    std::vector<std::string> all_ids;
    
    for (const auto& cfg : base_obstacles_) {
        if (std::find(all_ids.begin(), all_ids.end(), cfg.id) == all_ids.end()) {
            all_ids.push_back(cfg.id);
        }
    }
    for (const auto& cfg : extended_obstacles_) {
        if (std::find(all_ids.begin(), all_ids.end(), cfg.id) == all_ids.end()) {
            all_ids.push_back(cfg.id);
        }
    }
    for (const auto& cfg : custom_test_obstacles_) {
        if (std::find(all_ids.begin(), all_ids.end(), cfg.id) == all_ids.end()) {
            all_ids.push_back(cfg.id);
        }
    }
    
    if (all_ids.empty()) {
        RCLCPP_INFO(node_->get_logger(), "没有障碍物需要删除");
        return true;
    }
    
    RCLCPP_INFO(node_->get_logger(), "正在删除 %zu 个所有障碍物", all_ids.size());
    return removeMultipleCollisionObjects(all_ids);
}

void ObstacleManager::setConfigType(ConfigType config_type) {
    switch (config_type) {
        case ConfigType::EXTENDED:
            obstacles_config_ = extended_obstacles_;
            RCLCPP_INFO(node_->get_logger(), "已切换到扩展配置，包含 %zu 个障碍物", obstacles_config_.size());
            break;
        case ConfigType::CUSTOM:
            obstacles_config_ = custom_test_obstacles_;
            RCLCPP_INFO(node_->get_logger(), "已切换到自定义配置，包含 %zu 个障碍物", obstacles_config_.size());
            break;
        default: // ConfigType::DEFAULT
            obstacles_config_ = base_obstacles_;
            RCLCPP_INFO(node_->get_logger(), "已切换到默认配置，包含 %zu 个障碍物", obstacles_config_.size());
            break;
    }
}

void ObstacleManager::resetToDefault() {
    obstacles_config_ = base_obstacles_;
    RCLCPP_INFO(node_->get_logger(), "已重置为基础配置（box_1, box_2, box_3）");
}

void ObstacleManager::setObstacleConfig(const std::vector<ObstacleConfig>& new_config) {
    obstacles_config_ = new_config;
    RCLCPP_INFO(node_->get_logger(), "已设置新配置，包含 %zu 个障碍物", new_config.size());
}

bool ObstacleManager::checkObstacleExists(const std::string& object_id) {
    // 尝试静默删除
    bool result = removeCollisionObject(object_id, true);
    // 如果删除成功，说明存在，但我们删除掉了
    // 所以需要重新添加（如果测试需要的话）
    return result;
}

std::vector<std::string> ObstacleManager::getCurrentObstacleIds() const {
    std::vector<std::string> ids;
    for (const auto& cfg : obstacles_config_) {
        ids.push_back(cfg.id);
    }
    return ids;
}

std::vector<std::string> ObstacleManager::getAllObstacleIds() const {
    std::vector<std::string> all_ids;
    
    for (const auto& cfg : base_obstacles_) {
        if (std::find(all_ids.begin(), all_ids.end(), cfg.id) == all_ids.end()) {
            all_ids.push_back(cfg.id);
        }
    }
    for (const auto& cfg : extended_obstacles_) {
        if (std::find(all_ids.begin(), all_ids.end(), cfg.id) == all_ids.end()) {
            all_ids.push_back(cfg.id);
        }
    }
    for (const auto& cfg : custom_test_obstacles_) {
        if (std::find(all_ids.begin(), all_ids.end(), cfg.id) == all_ids.end()) {
            all_ids.push_back(cfg.id);
        }
    }
    
    return all_ids;
}

} // namespace cr7_robot_controller
