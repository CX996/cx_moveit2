#ifndef CR7_OBSTACLE_MANAGER_HPP
#define CR7_OBSTACLE_MANAGER_HPP

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace cr7_robot_controller {

/**
 * @class ObstacleManager
 * @brief 障碍物管理器，用于在MoveIt规划场景中添加和删除碰撞障碍物
 * 
 * 该类提供了在MoveIt规划场景中添加和删除碰撞障碍物的功能，
 * 支持多种障碍物类型（立方体、圆柱体、球体）和多种配置。
 */
class ObstacleManager {
public:
    /**
     * @struct ObstacleConfig
     * @brief 障碍物配置结构体
     */
    struct ObstacleConfig {
        std::string id;                  // 障碍物唯一标识符
        shape_msgs::msg::SolidPrimitive type;  // 障碍物类型
        std::vector<double> dimensions;  // 障碍物尺寸
        geometry_msgs::msg::Pose pose;   // 障碍物位置和姿态
    };

    /**
     * @enum ConfigType
     * @brief 配置类型枚举
     */
    enum class ConfigType {
        DEFAULT,    // 默认配置（基础障碍物）
        EXTENDED,   // 扩展配置（更多障碍物）
        CUSTOM      // 自定义配置
    };

    /**
     * @brief 构造函数
     * @param node ROS节点指针
     * @param world_frame 障碍物参考坐标系
     * @param auto_add 是否自动添加障碍物
     * @param config_type 配置类型
     */
    ObstacleManager(const rclcpp::Node::SharedPtr& node, 
                   const std::string& world_frame = "dummy_link",
                   bool auto_add = true,
                   ConfigType config_type = ConfigType::DEFAULT);

    /**
     * @brief 析构函数
     */
    ~ObstacleManager();

    /**
     * @brief 添加配置中的所有碰撞障碍物到规划场景
     * @param config_override 临时覆盖配置，用于一次性的添加操作
     * @param force 是否强制添加，即使可能已存在
     * @return 操作成功返回true，失败返回false
     */
    bool addCollisionObjects(const std::vector<ObstacleConfig>& config_override = {}, bool force = false);

    /**
     * @brief 从规划场景中删除指定ID的障碍物
     * @param object_id 要删除的障碍物ID
     * @param silent 静默模式，失败时不记录错误日志
     * @return 操作成功返回true，失败返回false
     */
    bool removeCollisionObject(const std::string& object_id, bool silent = false);

    /**
     * @brief 批量删除多个障碍物
     * @param object_ids 要删除的障碍物ID列表
     * @param silent 静默模式
     * @return 所有操作成功返回true，任意失败返回false
     */
    bool removeMultipleCollisionObjects(const std::vector<std::string>& object_ids, bool silent = false);

    /**
     * @brief 删除指定ID列表的障碍物
     * @param object_ids 要删除的障碍物ID列表
     * @param silent 静默模式
     * @return 所有操作成功返回true，任意失败返回false
     */
    bool removeObstaclesByIds(const std::vector<std::string>& object_ids, bool silent = false);

    /**
     * @brief 清理场景中的所有障碍物
     * @param obstacle_ids 要清理的障碍物ID列表，如果为nullptr，则尝试清理所有已知的障碍物
     * @return 清理成功返回true
     */
    bool cleanupScene(const std::vector<std::string>* obstacle_ids = nullptr);

    /**
     * @brief 删除配置中定义的所有障碍物
     * @return 所有操作成功返回true，任意失败返回false
     */
    bool removeAllConfiguredObstacles();

    /**
     * @brief 删除基础障碍物（box_1, box_2, box_3）
     * @return 操作成功返回true，失败返回false
     */
    bool removeBaseObstacles();

    /**
     * @brief 删除所有已知的障碍物
     * @return 操作成功返回true，失败返回false
     */
    bool removeAllObstacles();

    /**
     * @brief 设置配置类型
     * @param config_type 配置类型
     */
    void setConfigType(ConfigType config_type);

    /**
     * @brief 重置为默认配置（基础障碍物）
     */
    void resetToDefault();

    /**
     * @brief 设置新的障碍物配置
     * @param new_config 新的障碍物配置列表
     */
    void setObstacleConfig(const std::vector<ObstacleConfig>& new_config);

    /**
     * @brief 检查障碍物是否在场景中存在
     * @param object_id 障碍物ID
     * @return 如果存在返回true
     */
    bool checkObstacleExists(const std::string& object_id);

    /**
     * @brief 获取当前配置中的障碍物ID列表
     * @return 当前配置的障碍物ID列表
     */
    std::vector<std::string> getCurrentObstacleIds() const;

    /**
     * @brief 获取所有已知障碍物ID列表
     * @return 所有已知障碍物ID列表
     */
    std::vector<std::string> getAllObstacleIds() const;

private:
    /**
     * @brief 初始化默认障碍物配置
     */
    void initializeObstacleConfigs();

    rclcpp::Node::SharedPtr node_;                      // ROS节点指针
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;  // 规划场景接口
    std::string world_frame_;                           // 障碍物参考坐标系
    std::vector<ObstacleConfig> base_obstacles_;        // 基础障碍物配置
    std::vector<ObstacleConfig> extended_obstacles_;    // 扩展障碍物配置
    std::vector<ObstacleConfig> custom_test_obstacles_; // 自定义测试障碍物配置
    std::vector<ObstacleConfig> obstacles_config_;      // 当前障碍物配置
    rclcpp::TimerBase::SharedPtr timer_;                // 定时器，用于自动添加障碍物
};

} // namespace cr7_robot_controller

#endif // CR7_OBSTACLE_MANAGER_HPP
