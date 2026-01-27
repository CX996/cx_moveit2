"""
@file cr7_controller.launch.py
@brief CR7机器人控制器启动文件（参数方式）
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'test_mode', default_value='false',
            description='是否运行测试模式'
        ),
        DeclareLaunchArgument(
            'debug_mode', default_value='true',
            description='是否启用调试输出'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='是否使用仿真时间'
        ),

        # 打印模式信息
        LogInfo(msg=['运行模式: ', LaunchConfiguration('test_mode')]),
        LogInfo(msg=['调试模式: ', LaunchConfiguration('debug_mode')]),

        # 启动控制器节点
        Node(
            package='cr7_robot_controller',
            executable='cr7_controller',
            name='cr7_controller',
            output='screen',
            parameters=[{
                'test_mode': LaunchConfiguration('test_mode'),
                'debug_mode': LaunchConfiguration('debug_mode'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
