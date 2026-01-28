#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
测试发布者节点
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import numpy as np

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_joint_publisher')
        self.publisher = self.create_publisher(JointState, "/joint_states_robot", 10)
        self.timer = self.create_timer(0.1, self.publish_joints)  # 10Hz
        self.count = 0
        
    def publish_joints(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        
        self.get_logger().info(msg.header.stamp.__str__())

        # 6个关节名称
        msg.name = [f"test_joint_{i+1}" for i in range(6)]
        
        # 生成测试数据
        base_angle = self.count * 0.01
        msg.position = [
            np.sin(base_angle),           # 关节1
            np.sin(base_angle + 1.0),     # 关节2
            np.sin(base_angle + 2.0),     # 关节3
            np.sin(base_angle + 3.0),     # 关节4
            np.sin(base_angle + 4.0),     # 关节5
            np.sin(base_angle + 5.0)      # 关节6
        ]
        
        msg.velocity = [0.1] * 6
        msg.effort = [0.0] * 6
        
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {[f'{x:.3f}' for x in msg.position]}")
        self.count += 1

def main():
    rclpy.init()
    node = TestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Test publisher shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()