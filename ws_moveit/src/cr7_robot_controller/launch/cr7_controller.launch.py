#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'interactive_mode',
            default_value='false',
            description='是否启用交互模式'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='是否启用调试模式'
        ),
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='是否启用测试模式'
        ),
        DeclareLaunchArgument(
            'pilz_test',
            default_value='false',
            description='是否启用PILZ测试模式'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='是否启用仿真时间'
        ),       
        
        # CR7机器人控制器节点
        Node(
            package='cr7_robot_controller',
            executable='cr7_controller',
            name='cr7_controller',
            output='screen',
            parameters=[{
                'interactive_mode': LaunchConfiguration('interactive_mode'),
                'debug_mode': LaunchConfiguration('debug_mode'),
                'test_mode': LaunchConfiguration('test_mode'),
                'pilz_test': LaunchConfiguration('pilz_test'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])