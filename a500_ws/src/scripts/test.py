#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class AddFloorNode(Node):

    def __init__(self):
        super().__init__('add_floor_collision')

        # 发布 planning_scene
        self.scene_pub = self.create_publisher(
            PlanningScene,
            'planning_scene',
            10
        )

        # 延迟发布，确保 move_group 已启动
        self.timer = self.create_timer(2.0, self.publish_floor)

    def publish_floor(self):
        scene = PlanningScene()
        scene.is_diff = True  # ⚠️ 必须为 True（增量更新）

        floor = CollisionObject()
        floor.id = "floor"
        floor.header.frame_id = "world"  # ⚠️ world 坐标系

        # ========= 地面几何 =========
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            10.0,  # X
            10.0,  # Y
            0.02   # 厚度
        ]
        # ============================

        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = -0.01  # 上表面刚好是 Z=0

        floor.primitives.append(primitive)
        floor.primitive_poses.append(pose)
        floor.operation = CollisionObject.ADD

        scene.world.collision_objects.append(floor)

        self.scene_pub.publish(scene)
        self.get_logger().info("Floor collision object published.")

        # 只需要发布一次
        self.timer.cancel()


def main():
    rclpy.init()
    node = AddFloorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
