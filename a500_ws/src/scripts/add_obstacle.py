#!/usr/bin/env python3
"""
障碍物管理节点
这个节点提供了在MoveIt规划场景中添加和删除碰撞障碍物的功能。
通过ApplyPlanningScene服务与MoveIt进行交互，支持添加和删除障碍物操作。

主要功能：
1. 添加多个障碍物到规划场景
2. 从规划场景中删除指定的障碍物
3. 管理障碍物的生命周期

使用方式：
可以通过修改配置参数来控制添加的障碍物，或调用删除接口移除障碍物。

注意：
- 障碍物ID必须是唯一的
- 所有操作都是非阻塞的，通过ROS服务调用实现
"""

import rclpy
from rclpy.node import Node
import time
import copy
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene


class ObstacleManager(Node):
    """障碍物管理节点类
    
    这个类负责管理MoveIt规划场景中的碰撞障碍物，提供添加和删除功能。
    通过ROS服务与MoveIt的ApplyPlanningScene服务进行通信。
    
    属性：
        client: ApplyPlanningScene服务客户端
        world_frame: 障碍物参考坐标系
        obstacles_config: 障碍物配置列表
        default_obstacles: 默认障碍物配置备份
    """
    
    def __init__(self, auto_add=True, config_type="default"):
        """初始化障碍物管理节点
        
        初始化ROS节点，创建服务客户端，设置配置参数，并启动定时器添加初始障碍物。
        
        参数：
            auto_add (bool): 是否自动添加障碍物，默认为True
            config_type (str): 配置类型，可选 "default"（3个默认障碍物）、"extended"（更多障碍物）、"custom"（自定义）
        """
        # 初始化父类Node，节点名称为'obstacle_manager_node'
        super().__init__('obstacle_manager_node')

        # 创建ApplyPlanningScene服务客户端
        # 这个服务用于将规划场景更改应用到MoveIt规划场景中
        self.client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

        # 等待服务可用
        self.get_logger().info('等待 /apply_planning_scene 服务...')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('服务等待超时！')
            raise RuntimeError('无法连接到ApplyPlanningScene服务')

        self.get_logger().info('服务连接成功')

        # ===============================
        # 基础障碍物配置（您指定的三个障碍物）
        # ===============================
        self.base_obstacles = [
            {
                "id": "box_1",  # 障碍物唯一标识符
                "type": SolidPrimitive.BOX,  # 障碍物类型：立方体
                "dimensions": [1.0, 0.01, 0.5],  # 尺寸：[长, 宽, 高]（单位：米）
                "pose": {  # 障碍物位置
                    "x": 0.5,  # X坐标
                    "y": 0.4,  # Y坐标
                    "z": 0.25,  # Z坐标
                }
            },
            {
                "id": "box_2",
                "type": SolidPrimitive.BOX,
                "dimensions": [1.0, 0.01, 0.5],
                "pose": {
                    "x": 0.5,
                    "y": -0.4,
                    "z": 0.25,
                }
            },
            {
                "id": "box_3",
                "type": SolidPrimitive.BOX,
                "dimensions": [0.01, 0.8, 0.5],
                "pose": {
                    "x": 1.0,
                    "y": 0.0,
                    "z": 0.25,
                }
            },
            {
                "id": "box_4",
                "type": SolidPrimitive.BOX,
                "dimensions": [1.1, 0.9, 0.01],
                "pose": {
                    "x": 0.45,
                    "y": 0.0,
                    "z": -0.005,
                }
            },
        ]

        # ===============================
        # 扩展障碍物配置（更多障碍物用于测试）
        # ===============================
        self.extended_obstacles = self.base_obstacles + [
            {
                "id": "box_4",
                "type": SolidPrimitive.BOX,
                "dimensions": [0.5, 0.01, 0.3],
                "pose": {
                    "x": 0.1,
                    "y": 0.6,
                    "z": 0.15,
                }
            },
            {
                "id": "box_5",
                "type": SolidPrimitive.BOX,
                "dimensions": [0.5, 0.01, 0.3],
                "pose": {
                    "x": 0.1,
                    "y": -0.6,
                    "z": 0.15,
                }
            },
            {
                "id": "box_6",
                "type": SolidPrimitive.BOX,
                "dimensions": [0.01, 0.5, 0.3],
                "pose": {
                    "x": 0.9,
                    "y": 0.0,
                    "z": 0.15,
                }
            },
        ]

        # ===============================
        # 自定义测试障碍物配置
        # ===============================
        self.custom_test_obstacles = [
            {
                "id": "test_cylinder_1",
                "type": SolidPrimitive.CYLINDER,
                "dimensions": [0.1, 0.05],  # 高度, 半径
                "pose": {
                    "x": 0.4,
                    "y": 0.0,
                    "z": 0.05,
                }
            },
            {
                "id": "test_sphere_1",
                "type": SolidPrimitive.SPHERE,
                "dimensions": [0.08],  # 半径
                "pose": {
                    "x": 0.5,
                    "y": 0.3,
                    "z": 0.08,
                }
            },
            {
                "id": "test_box_small",
                "type": SolidPrimitive.BOX,
                "dimensions": [0.1, 0.1, 0.1],
                "pose": {
                    "x": 0.5,
                    "y": -0.3,
                    "z": 0.05,
                }
            },
        ]

        # 根据配置类型设置当前障碍物配置
        if config_type == "extended":
            self.default_obstacles = copy.deepcopy(self.extended_obstacles)
        elif config_type == "custom":
            self.default_obstacles = copy.deepcopy(self.custom_test_obstacles)
        else:  # "default"
            self.default_obstacles = copy.deepcopy(self.base_obstacles)

        # 当前障碍物配置
        self.obstacles_config = copy.deepcopy(self.default_obstacles)

        # 统一使用的参考坐标系
        # 所有障碍物都将相对于此坐标系放置
        self.world_frame = "dummy_link"  # 根据你的机器人修改此坐标系

        # 如果需要自动添加障碍物，则启动定时器
        if auto_add:
            self.get_logger().info(f'将在2秒后自动添加 {len(self.obstacles_config)} 个障碍物...')
            self.timer = self.create_timer(2.0, self.add_collision_objects)
        else:
            self.get_logger().info('自动添加已禁用，请手动调用add_collision_objects()')
            self.timer = None

    # ==========================================================
    # 核心接口：一次性添加所有配置的碰撞物体
    # ==========================================================
    def add_collision_objects(self, config_override=None, force=False):
        """添加配置中的所有碰撞障碍物到规划场景
        
        从obstacles_config读取配置，创建碰撞物体，并通过服务调用添加到规划场景。
        此方法只会执行一次，然后取消定时器。
        
        参数：
            config_override (list, optional): 临时覆盖配置，用于一次性的添加操作
            force (bool): 是否强制添加，即使可能已存在
            
        返回：
            bool: 操作成功返回True，失败返回False
        """
        try:
            # 如果定时器存在，先取消
            if self.timer and self.timer.is_callback_ready:
                self.timer.cancel()
                self.get_logger().debug('定时器已取消')

            # 确定使用哪个配置
            if config_override is not None:
                current_config = config_override
                self.get_logger().info(f'使用覆盖配置，添加 {len(current_config)} 个障碍物')
            else:
                current_config = self.obstacles_config
                self.get_logger().info(f'使用当前配置，添加 {len(current_config)} 个障碍物')

            if not current_config:
                self.get_logger().warn('配置为空，不添加任何障碍物')
                return True

            # 创建碰撞物体列表
            collision_objects = []

            # 遍历所有障碍物配置，创建碰撞物体
            for cfg in current_config:
                obj = self.create_primitive_object(cfg)
                if obj:
                    collision_objects.append(obj)
                else:
                    self.get_logger().warn(f"无法创建障碍物 {cfg.get('id', '未知')}，跳过")

            if not collision_objects:
                self.get_logger().error('没有有效的障碍物可以添加')
                return False

            # 创建规划场景消息
            planning_scene = self.create_planning_scene(collision_objects)

            # 创建服务请求
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene

            # 异步调用服务
            future = self.client.call_async(request)
            
            # 等待服务响应
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            # 处理服务响应
            if future.result() is not None:
                self.get_logger().info(
                    f"✅ 成功添加 {len(collision_objects)} 个障碍物"
                )
                return True
            else:
                if force:
                    # 如果是强制模式，即使失败也继续
                    self.get_logger().warn("⚠️ 添加障碍物可能已存在，继续执行")
                    return True
                else:
                    self.get_logger().error("❌ 添加障碍物失败：服务调用失败")
                    return False

        except Exception as e:
            # 记录异常信息
            self.get_logger().error(f"添加障碍物异常: {str(e)}")
            return False

    # ==========================================================
    # 删除指定ID的障碍物
    # ==========================================================
    def remove_collision_object(self, object_id: str, silent=False):
        """从规划场景中删除指定ID的障碍物
        
        参数：
            object_id (str): 要删除的障碍物ID
            silent (bool): 静默模式，失败时不记录错误日志
            
        返回：
            bool: 操作成功返回True，失败返回False
        """
        try:
            # 创建删除操作的碰撞物体
            collision_object = CollisionObject()
            collision_object.id = object_id
            collision_object.header.frame_id = self.world_frame
            collision_object.operation = CollisionObject.REMOVE  # 设置操作为删除

            # 创建规划场景消息
            planning_scene = PlanningScene()
            planning_scene.is_diff = True
            planning_scene.robot_state.is_diff = True
            planning_scene.world.collision_objects.append(collision_object)

            # 创建服务请求
            request = ApplyPlanningScene.Request()
            request.scene = planning_scene

            # 异步调用服务
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            # 处理响应
            if future.result() is not None:
                if not silent:
                    self.get_logger().info(f"✅ 成功删除障碍物 '{object_id}'")
                return True
            else:
                if not silent:
                    self.get_logger().warn(f"⚠️ 删除障碍物 '{object_id}' 失败：可能不存在")
                return False
                
        except Exception as e:
            # 记录异常信息
            if not silent:
                self.get_logger().error(f"删除障碍物异常: {str(e)}")
            return False

    # ==========================================================
    # 批量删除多个障碍物
    # ==========================================================
    def remove_multiple_collision_objects(self, object_ids: list, silent=False):
        """批量删除多个障碍物
        
        参数：
            object_ids (list): 要删除的障碍物ID列表
            silent (bool): 静默模式
            
        返回：
            bool: 所有操作成功返回True，任意失败返回False
        """
        success = True
        
        for obj_id in object_ids:
            if not self.remove_collision_object(obj_id, silent=silent):
                success = False
                
        return success

    # ==========================================================
    # 删除指定ID列表的障碍物
    # ==========================================================
    def remove_obstacles_by_ids(self, object_ids: list, silent=False):
        """删除指定ID列表的障碍物（不依赖于当前配置）
        
        参数：
            object_ids (list): 要删除的障碍物ID列表
            silent (bool): 静默模式
            
        返回：
            bool: 所有操作成功返回True，任意失败返回False
        """
        if not object_ids:
            if not silent:
                self.get_logger().info("没有指定要删除的障碍物")
            return True
            
        if not silent:
            self.get_logger().info(f"正在删除 {len(object_ids)} 个指定的障碍物: {object_ids}")
        return self.remove_multiple_collision_objects(object_ids, silent=silent)

    # ==========================================================
    # 清理场景中的所有障碍物
    # ==========================================================
    def cleanup_scene(self, obstacle_ids=None):
        """清理规划场景中的障碍物
        
        参数：
            obstacle_ids (list, optional): 要清理的障碍物ID列表
                如果为None，则尝试清理所有已知的障碍物
                
        返回：
            bool: 清理成功返回True
        """
        if obstacle_ids is None:
            # 尝试清理所有已知障碍物
            all_known_ids = []
            # 添加基础障碍物
            for obstacle in self.base_obstacles:
                all_known_ids.append(obstacle["id"])
            # 添加扩展障碍物
            for obstacle in self.extended_obstacles:
                if obstacle["id"] not in all_known_ids:
                    all_known_ids.append(obstacle["id"])
            # 添加自定义测试障碍物
            for obstacle in self.custom_test_obstacles:
                all_known_ids.append(obstacle["id"])
        else:
            all_known_ids = obstacle_ids
            
        self.get_logger().info(f"清理场景中的障碍物: {all_known_ids}")
        
        # 静默模式删除，不记录错误
        self.remove_multiple_collision_objects(all_known_ids, silent=True)
        
        # 等待一下确保删除完成
        time.sleep(0.5)
        
        self.get_logger().info("场景清理完成")
        return True

    # ==========================================================
    # 删除所有配置中的障碍物
    # ==========================================================
    def remove_all_configured_obstacles(self):
        """删除配置中定义的所有障碍物
        
        从obstacles_config中读取所有障碍物ID，并批量删除它们。
        
        返回：
            bool: 所有操作成功返回True，任意失败返回False
        """
        # 从配置中提取所有障碍物ID
        object_ids = [cfg["id"] for cfg in self.obstacles_config]
        
        if not object_ids:
            self.get_logger().info("没有配置的障碍物需要删除")
            return True
            
        self.get_logger().info(f"正在删除 {len(object_ids)} 个配置的障碍物: {object_ids}")
        return self.remove_multiple_collision_objects(object_ids)

    # ==========================================================
    # 删除基础障碍物（您指定的三个）
    # ==========================================================
    def remove_base_obstacles(self):
        """删除基础障碍物（您指定的三个：box_1, box_2, box_3）
        
        返回：
            bool: 操作成功返回True，失败返回False
        """
        base_ids = ["box_1", "box_2", "box_3"]
        
        self.get_logger().info(f"正在删除基础障碍物: {base_ids}")
        return self.remove_multiple_collision_objects(base_ids)

    # ==========================================================
    # 删除所有障碍物
    # ==========================================================
    def remove_all_obstacles(self):
        """删除所有已知的障碍物
        
        返回：
            bool: 操作成功返回True，失败返回False
        """
        # 收集所有可能的ID
        all_ids = set()
        for cfg in self.base_obstacles:
            all_ids.add(cfg["id"])
        for cfg in self.extended_obstacles:
            all_ids.add(cfg["id"])
        for cfg in self.custom_test_obstacles:
            all_ids.add(cfg["id"])
        
        all_ids = list(all_ids)
        
        if not all_ids:
            self.get_logger().info("没有障碍物需要删除")
            return True
            
        self.get_logger().info(f"正在删除 {len(all_ids)} 个所有障碍物: {all_ids}")
        return self.remove_multiple_collision_objects(all_ids)

    # ==========================================================
    # 创建基本几何体对象
    # ==========================================================
    def create_primitive_object(self, cfg: dict) -> CollisionObject:
        """创建立方体类型的碰撞物体
        
        根据配置字典创建CollisionObject消息。支持多种几何体类型。
        
        参数：
            cfg (dict): 障碍物配置字典，包含以下键：
                - id: 障碍物唯一ID
                - type: 障碍物类型（SolidPrimitive类型）
                - dimensions: 障碍物尺寸列表
                - pose: 位置字典，包含x, y, z坐标
        
        返回：
            CollisionObject: 配置好的碰撞物体消息
        """
        try:
            # 创建碰撞物体对象
            collision_object = CollisionObject()
            
            # 设置障碍物ID（必须唯一）
            collision_object.id = cfg["id"]
            
            # 设置参考坐标系
            collision_object.header.frame_id = self.world_frame
            
            # 设置操作为添加
            collision_object.operation = CollisionObject.ADD

            # 创建基本几何体
            primitive = SolidPrimitive()
            primitive.type = cfg["type"]  # 设置类型
            
            # 检查尺寸
            dimensions = cfg.get("dimensions", [])
            if cfg["type"] == SolidPrimitive.BOX and len(dimensions) != 3:
                self.get_logger().error(f"BOX类型需要3个尺寸参数，但得到 {len(dimensions)} 个")
                return None
            elif cfg["type"] == SolidPrimitive.SPHERE and len(dimensions) != 1:
                self.get_logger().error(f"SPHERE类型需要1个尺寸参数，但得到 {len(dimensions)} 个")
                return None
            elif cfg["type"] == SolidPrimitive.CYLINDER and len(dimensions) != 2:
                self.get_logger().error(f"CYLINDER类型需要2个尺寸参数，但得到 {len(dimensions)} 个")
                return None
                
            primitive.dimensions = dimensions  # 设置尺寸

            # 创建位姿
            pose = Pose()
            pose.position.x = cfg["pose"].get("x", 0.0)  # 获取X坐标，默认为0.0
            pose.position.y = cfg["pose"].get("y", 0.0)  # 获取Y坐标，默认为0.0
            pose.position.z = cfg["pose"].get("z", 0.0)  # 获取Z坐标，默认为0.0
            pose.orientation.w = 1.0  # 默认无旋转（单位四元数）

            # 将基本几何体和位姿添加到碰撞物体
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)

            return collision_object
            
        except KeyError as e:
            self.get_logger().error(f"配置缺少必要字段: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"创建障碍物对象失败: {e}")
            return None

    # ==========================================================
    # 封装PlanningScene（diff方式，安全）
    # ==========================================================
    def create_planning_scene(self, objects: list) -> PlanningScene:
        """创建规划场景消息
        
        创建PlanningScene消息，用于向MoveIt发送场景更新。
        使用diff方式，只发送变化的部分，而不是整个场景。
        
        参数：
            objects (list): CollisionObject对象列表
            
        返回：
            PlanningScene: 配置好的规划场景消息
        """
        # 创建规划场景
        scene = PlanningScene()
        
        # 设置为diff模式，只发送变化的部分
        # 这是推荐的方式，避免发送整个场景状态
        scene.is_diff = True
        
        # 机器人状态也设置为diff模式
        scene.robot_state.is_diff = True

        # 添加所有碰撞物体到世界场景
        for obj in objects:
            scene.world.collision_objects.append(obj)

        return scene

    # ==========================================================
    # 配置管理方法
    # ==========================================================
    def set_config_type(self, config_type: str):
        """设置配置类型
        
        参数：
            config_type (str): 配置类型，可选 "default"、"extended"、"custom"
        """
        if config_type == "extended":
            self.obstacles_config = copy.deepcopy(self.extended_obstacles)
            self.get_logger().info(f'已切换到扩展配置，包含 {len(self.obstacles_config)} 个障碍物')
        elif config_type == "custom":
            self.obstacles_config = copy.deepcopy(self.custom_test_obstacles)
            self.get_logger().info(f'已切换到自定义配置，包含 {len(self.obstacles_config)} 个障碍物')
        else:  # "default"
            self.obstacles_config = copy.deepcopy(self.base_obstacles)
            self.get_logger().info(f'已切换到默认配置，包含 {len(self.obstacles_config)} 个障碍物')

    def reset_to_default(self):
        """重置为默认配置（基础障碍物）"""
        self.obstacles_config = copy.deepcopy(self.base_obstacles)
        self.get_logger().info('已重置为基础配置（box_1, box_2, box_3）')

    def set_obstacle_config(self, new_config):
        """设置新的障碍物配置
        
        参数：
            new_config (list): 新的障碍物配置列表
        """
        self.obstacles_config = copy.deepcopy(new_config)
        self.get_logger().info(f'已设置新配置，包含 {len(new_config)} 个障碍物')

    # ==========================================================
    # 检查障碍物是否存在
    # ==========================================================
    def check_obstacle_exists(self, object_id: str):
        """检查障碍物是否在场景中存在
        
        注意：这是一个简单检查，通过尝试删除来判断
        实际上MoveIt没有提供直接的查询接口
        
        参数：
            object_id (str): 障碍物ID
            
        返回：
            bool: 如果存在返回True
        """
        # 尝试静默删除
        result = self.remove_collision_object(object_id, silent=True)
        # 如果删除成功，说明存在，但我们删除掉了
        # 所以需要重新添加（如果测试需要的话）
        return result

    # ==========================================================
    # 信息查询方法
    # ==========================================================
    def get_current_obstacle_ids(self):
        """获取当前配置中的障碍物ID列表
        
        返回：
            list: 当前配置的障碍物ID列表
        """
        return [cfg["id"] for cfg in self.obstacles_config]

    def get_all_obstacle_ids(self):
        """获取所有已知障碍物ID列表
        
        返回：
            list: 所有已知障碍物ID列表
        """
        all_ids = set()
        for cfg in self.base_obstacles:
            all_ids.add(cfg["id"])
        for cfg in self.extended_obstacles:
            all_ids.add(cfg["id"])
        for cfg in self.custom_test_obstacles:
            all_ids.add(cfg["id"])
        return list(all_ids)

    # ==========================================================
    # 清理方法
    # ==========================================================
    def cleanup(self):
        """清理资源"""
        if self.timer and self.timer.is_callback_ready:
            self.timer.cancel()
        self.get_logger().info('资源清理完成')


def interactive_test():
    """交互式测试程序，让用户完全控制何时添加和删除障碍物"""
    print("\n" + "="*60)
    print("交互式障碍物管理测试")
    print("="*60)
    
    rclpy.init()
    
    try:
        # 创建障碍物管理器，不自动添加
        manager = ObstacleManager(auto_add=False, config_type="default")
        
        # 初始清理场景
        print("\n1. 初始清理场景...")
        manager.cleanup_scene()
        time.sleep(1)
        
        while True:
            print("\n" + "="*40)
            print("当前选项:")
            print("1. 添加障碍物")
            print("2. 删除障碍物")
            print("3. 切换配置")
            print("4. 清理场景")
            print("5. 显示当前配置")
            print("6. 显示所有已知障碍物")
            print("0. 退出")
            print("="*40)
            
            choice = input("请选择 (0-6): ").strip()
            
            if choice == '0':
                print("退出交互式测试")
                break
                
            elif choice == '1':
                print("\n--- 添加障碍物 ---")
                print("1. 添加当前配置的所有障碍物")
                print("2. 只添加基础障碍物 (box_1, box_2, box_3)")
                print("3. 只添加指定ID的障碍物")
                sub_choice = input("请选择 (1-3): ").strip()
                
                if sub_choice == '1':
                    print(f"添加当前配置的 {len(manager.obstacles_config)} 个障碍物...")
                    if manager.add_collision_objects(force=True):
                        print("✅ 添加成功")
                    else:
                        print("❌ 添加失败")
                elif sub_choice == '2':
                    print("添加基础障碍物 (box_1, box_2, box_3)...")
                    if manager.add_collision_objects(manager.base_obstacles, force=True):
                        print("✅ 添加成功")
                    else:
                        print("❌ 添加失败")
                elif sub_choice == '3':
                    obj_id = input("请输入障碍物ID: ").strip()
                    # 查找对应配置
                    obj_config = None
                    for cfg in manager.obstacles_config:
                        if cfg["id"] == obj_id:
                            obj_config = [cfg]
                            break
                    
                    if obj_config:
                        print(f"添加障碍物 '{obj_id}'...")
                        if manager.add_collision_objects(obj_config, force=True):
                            print("✅ 添加成功")
                        else:
                            print("❌ 添加失败")
                    else:
                        print(f"未找到障碍物 '{obj_id}' 的配置")
                else:
                    print("无效选择")
                    
            elif choice == '2':
                print("\n--- 删除障碍物 ---")
                print("1. 删除当前配置的所有障碍物")
                print("2. 只删除基础障碍物 (box_1, box_2, box_3)")
                print("3. 删除指定ID的障碍物")
                print("4. 删除所有已知障碍物")
                sub_choice = input("请选择 (1-4): ").strip()
                
                if sub_choice == '1':
                    print(f"删除当前配置的 {len(manager.obstacles_config)} 个障碍物...")
                    if manager.remove_all_configured_obstacles():
                        print("✅ 删除成功")
                    else:
                        print("❌ 删除失败")
                elif sub_choice == '2':
                    print("删除基础障碍物 (box_1, box_2, box_3)...")
                    if manager.remove_base_obstacles():
                        print("✅ 删除成功")
                    else:
                        print("❌ 删除失败")
                elif sub_choice == '3':
                    obj_id = input("请输入障碍物ID: ").strip()
                    print(f"删除障碍物 '{obj_id}'...")
                    if manager.remove_collision_object(obj_id):
                        print("✅ 删除成功")
                    else:
                        print("❌ 删除失败或障碍物不存在")
                elif sub_choice == '4':
                    print("删除所有已知障碍物...")
                    if manager.remove_all_obstacles():
                        print("✅ 删除成功")
                    else:
                        print("❌ 删除失败")
                else:
                    print("无效选择")
                    
            elif choice == '3':
                print("\n--- 切换配置 ---")
                print("1. 默认配置 (box_1, box_2, box_3)")
                print("2. 扩展配置 (基础 + 3个额外障碍物)")
                print("3. 自定义配置 (圆柱体、球体、小立方体)")
                sub_choice = input("请选择 (1-3): ").strip()
                
                if sub_choice == '1':
                    manager.set_config_type("default")
                elif sub_choice == '2':
                    manager.set_config_type("extended")
                elif sub_choice == '3':
                    manager.set_config_type("custom")
                else:
                    print("无效选择")
                    
            elif choice == '4':
                print("\n清理所有障碍物...")
                manager.cleanup_scene()
                print("✅ 场景已清理")
                
            elif choice == '5':
                print("\n--- 当前配置信息 ---")
                print(f"配置类型: {len(manager.obstacles_config)} 个障碍物")
                for i, cfg in enumerate(manager.obstacles_config, 1):
                    print(f"{i}. ID: {cfg['id']}, 类型: {cfg['type']}, "
                          f"尺寸: {cfg['dimensions']}, "
                          f"位置: ({cfg['pose']['x']:.2f}, {cfg['pose']['y']:.2f}, {cfg['pose']['z']:.2f})")
                    
            elif choice == '6':
                print("\n--- 所有已知障碍物ID ---")
                all_ids = manager.get_all_obstacle_ids()
                print(f"共 {len(all_ids)} 个障碍物:")
                for i, obj_id in enumerate(sorted(all_ids), 1):
                    print(f"{i}. {obj_id}")
                    
            else:
                print("无效选择，请重新输入")
                
            time.sleep(0.5)  # 短暂暂停，让用户看清楚结果
            
    except KeyboardInterrupt:
        print("\n\n用户中断，清理场景...")
        if 'manager' in locals():
            manager.cleanup_scene()
    except Exception as e:
        print(f"\n❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'manager' in locals():
            manager.cleanup()
        rclpy.shutdown()
        print("\n测试程序已退出")


def test_specific_obstacles():
    """只测试您指定的三个障碍物"""
    print("\n" + "="*60)
    print("测试指定的三个障碍物 (box_1, box_2, box_3)")
    print("="*60)
    
    rclpy.init()
    
    try:
        # 创建障碍物管理器，使用默认配置
        manager = ObstacleManager(auto_add=False, config_type="default")
        
        # 清理场景
        print("\n1. 清理场景...")
        manager.cleanup_scene()
        time.sleep(1)
        
        # 添加基础障碍物
        print("\n2. 添加基础障碍物 (box_1, box_2, box_3)...")
        manager.reset_to_default()
        if manager.add_collision_objects(force=True):
            print("✅ 基础障碍物添加成功")
        time.sleep(2)
        
        # 显示当前障碍物
        print("\n3. 当前场景中的障碍物:")
        for cfg in manager.obstacles_config:
            print(f"  - {cfg['id']}")
            
        # 等待用户决定是否删除
        print("\n" + "-"*40)
        print("障碍物已添加，请进行测试...")
        print("-"*40)
        response = input("是否删除这些障碍物？(y/n): ").strip().lower()
        
        if response == 'y':
            print("\n4. 删除障碍物...")
            if manager.remove_base_obstacles():
                print("✅ 基础障碍物删除成功")
        else:
            print("\n障碍物保留在场景中")
            
    except KeyboardInterrupt:
        print("\n\n用户中断，清理场景...")
        if 'manager' in locals():
            manager.cleanup_scene()
    finally:
        if 'manager' in locals():
            manager.cleanup()
        rclpy.shutdown()


def main():
    """
    主函数 - 测试模式
    """
    print("=" * 60)
    print("障碍物管理器测试程序")
    print("=" * 60)
    print("\n请选择测试模式：")
    print("1. 交互式测试（完全控制添加/删除）")
    print("2. 只测试指定的三个障碍物")
    print("3. 快速测试（添加默认障碍物，然后清理）")
    print("0. 退出")
    print("=" * 60)
    
    choice = input("请选择 (0-3): ").strip()
    
    if choice == '1':
        interactive_test()
    elif choice == '2':
        test_specific_obstacles()
    elif choice == '3':
        # 快速测试
        print("\n快速测试：添加默认障碍物，等待5秒，然后清理")
        rclpy.init()
        try:
            manager = ObstacleManager(auto_add=False)
            manager.cleanup_scene()
            time.sleep(1)
            
            print("添加默认障碍物...")
            manager.reset_to_default()
            if manager.add_collision_objects(force=True):
                print("✅ 添加成功")
                
            print("等待5秒...")
            time.sleep(5)
            
            print("清理场景...")
            manager.cleanup_scene()
            print("✅ 场景已清理")
            
        finally:
            if 'manager' in locals():
                manager.cleanup()
            rclpy.shutdown()
    elif choice == '0':
        print("退出测试")
    else:
        print("无效选择，运行交互式测试")
        interactive_test()


if __name__ == '__main__':
    # 注意：在实际使用中，应该只导入ObstacleManager类
    # 这个main函数仅用于测试目的
    
    main()