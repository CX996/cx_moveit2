# 导入必要的Python标准库和ROS 2相关模块
import os  # Python操作系统接口模块，用于文件路径操作等
from launch import LaunchDescription  # ROS 2 launch系统的核心类，用于描述启动哪些节点和配置
from launch.actions import DeclareLaunchArgument  # 用于在launch文件中声明可配置的启动参数
from launch.substitutions import LaunchConfiguration  # 用于获取启动时传入的参数值
from launch.conditions import IfCondition, UnlessCondition  # 用于根据条件决定是否启动某个节点
from launch_ros.actions import Node  # ROS 2节点启动动作，用于启动具体的ROS节点
from launch.actions import ExecuteProcess  # 用于执行外部系统命令或程序
from ament_index_python.packages import get_package_share_directory  # 获取ROS包的安装共享目录路径
from moveit_configs_utils import MoveItConfigsBuilder  # MoveIt 2配置构建工具，简化MoveIt配置过程


# 定义生成launch描述的主函数，这是launch文件的入口点
def generate_launch_description():

    # 声明命令行参数：是否启用RViz教程模式
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"  # 参数名：rviz_tutorial，默认值：False，描述：教程标志
    )

    # 声明命令行参数：是否启动MongoDB数据库服务
    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"  # 参数名：db，默认值：False，描述：数据库标志
    )

    # 声明命令行参数：ROS 2控制硬件接口类型
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",  # 参数名
        default_value="mock_components",  # 默认值：模拟组件
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",  # 描述：硬件接口类型，可选值[模拟组件, isaac]
    )

    # 使用MoveIt配置构建器创建完整的MoveIt配置
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")  # 创建构建器，指定机器人模型为panda
        .robot_description(  # 配置机器人描述（URDF）
            file_path="config/panda.urdf.xacro",  # URDF文件路径，使用xacro格式
            mappings={  # 参数映射，将xacro中的变量替换为实际值
                "ros2_control_hardware_type": LaunchConfiguration(  # 映射硬件类型参数
                    "ros2_control_hardware_type"  # 对应上面声明的硬件类型参数
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")  # 添加语义描述文件（SRDF），包含运动学约束等信息
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")  # 配置轨迹执行参数和控制器
        .planning_pipelines(  # 配置可用的运动规划算法管道
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]  # 启用OMPL、CHOMP、Pilz三种规划器
        )
        .to_moveit_configs()  # 构建并返回完整的MoveIt配置对象
    )

    # 启动MoveIt的核心节点move_group，负责运动规划、碰撞检测等核心功能
    move_group_node = Node(
        package="moveit_ros_move_group",  # 节点所在的ROS包
        executable="move_group",  # 可执行文件名
        output="screen",  # 将节点输出打印到终端屏幕
        parameters=[moveit_config.to_dict()],  # 传递MoveIt配置参数
        arguments=["--ros-args", "--log-level", "info"],  # 设置ROS参数：日志级别为info
    )

    # 获取RViz教程模式参数的当前值
    tutorial_mode = LaunchConfiguration("rviz_tutorial")

    # 构建RViz配置文件的基础路径
    rviz_base = os.path.join(  # 拼接路径
        get_package_share_directory("moveit_resources_panda_moveit_config"), "launch"  # 获取panda_moveit_config包的launch目录
    )

    # 完整版RViz配置文件的完整路径
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    # 空版/教程版RViz配置文件的完整路径
    rviz_empty_config = os.path.join(rviz_base, "moveit_empty.rviz")

    # 教程模式下的RViz节点（使用简化界面）
    rviz_node_tutorial = Node(
        package="rviz2",  # RViz2包
        executable="rviz2",  # RViz2可执行文件
        name="rviz2",  # 节点名称
        output="log",  # 输出重定向到日志文件
        arguments=["-d", rviz_empty_config],  # 指定使用空配置文件启动
        parameters=[  # 传递给RViz的参数
            moveit_config.robot_description,  # 机器人描述
            moveit_config.robot_description_semantic,  # 机器人语义描述
            moveit_config.planning_pipelines,  # 规划管道配置
            moveit_config.robot_description_kinematics,  # 运动学参数
        ],
        condition=IfCondition(tutorial_mode),  # 条件：仅当tutorial_mode为True时启动
    )

    # 普通模式下的RViz节点（使用完整界面）
    rviz_node = Node(
        package="rviz2",  # RViz2包
        executable="rviz2",  # RViz2可执行文件
        name="rviz2",  # 节点名称
        output="log",  # 输出重定向到日志文件
        arguments=["-d", rviz_full_config],  # 指定使用完整配置文件启动
        parameters=[  # 传递给RViz的参数（与普通模式相同）
            moveit_config.robot_description,  # 机器人描述
            moveit_config.robot_description_semantic,  # 机器人语义描述
            moveit_config.planning_pipelines,  # 规划管道配置
            moveit_config.robot_description_kinematics,  # 运动学参数
        ],
        condition=UnlessCondition(tutorial_mode),  # 条件：仅当tutorial_mode为False时启动
    )

    # 静态坐标变换发布节点：发布world到panda_link0的固定变换关系
    static_tf_node = Node(
        package="tf2_ros",  # tf2_ros包
        executable="static_transform_publisher",  # 静态变换发布器
        name="static_transform_publisher",  # 节点名称
        output="log",  # 输出重定向到日志
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],  # 参数：x,y,z,roll,pitch,yaw,父坐标系,子坐标系
    )

    # 机器人状态发布器节点：发布机器人各关节状态和TF变换
    robot_state_publisher = Node(
        package="robot_state_publisher",  # robot_state_publisher包
        executable="robot_state_publisher",  # 机器人状态发布器
        name="robot_state_publisher",  # 节点名称
        output="both",  # 同时输出到屏幕和日志
        parameters=[moveit_config.robot_description],  # 传入机器人描述参数
    )

    # 构建ROS 2控制器配置文件的路径
    ros2_controllers_path = os.path.join(  # 拼接路径
        get_package_share_directory("moveit_resources_panda_moveit_config"),  # 获取panda_moveit_config包目录
        "config",  # config子目录
        "ros2_controllers.yaml",  # 控制器配置文件
    )

    # ROS 2控制节点：管理机器人控制器的生命周期
    ros2_control_node = Node(
        package="controller_manager",  # controller_manager包
        executable="ros2_control_node",  # ROS 2控制节点
        parameters=[ros2_controllers_path],  # 传入控制器配置参数
        remappings=[  # 话题重映射
            ("/controller_manager/robot_description", "/robot_description"),  # 将内部话题映射到全局话题
        ],
        output="screen",  # 输出到屏幕
    )

    # 关节状态广播器生成器：启动关节状态广播控制器
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",  # controller_manager包
        executable="spawner",  # 控制器生成器
        arguments=[  # 参数：控制器名称和管理器地址
            "joint_state_broadcaster",  # 要生成的控制器名称
            "--controller-manager",  # 指定选项
            "/controller_manager",  # 控制器管理器地址
        ],
    )

    # 熊猫机械臂控制器生成器：启动机械臂运动控制器
    panda_arm_controller_spawner = Node(
        package="controller_manager",  # controller_manager包
        executable="spawner",  # 控制器生成器
        arguments=["panda_arm_controller", 
                   "-c", 
                   "/controller_manager"],  # 参数：控制器名称和管理器地址
    )

    # 熊猫手爪控制器生成器：启动手爪开合控制器
    panda_hand_controller_spawner = Node(
        package="controller_manager",  # controller_manager包
        executable="spawner",  # 控制器生成器
        arguments=["panda_hand_controller", 
                   "-c", 
                   "/controller_manager"],  # 参数：控制器名称和管理器地址
    )

    # 获取数据库启动参数的当前值
    db_config = LaunchConfiguration("db")

    # MongoDB数据库服务节点：提供运动规划数据存储和检索服务
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",  # warehouse_ros_mongo包
        executable="mongo_wrapper_ros.py",  # MongoDB包装脚本
        parameters=[  # 数据库配置参数
            {"warehouse_port": 33829},  # 数据库端口号
            {"warehouse_host": "localhost"},  # 数据库主机地址
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},  # 数据库插件类型
        ],
        output="screen",  # 输出到屏幕
        condition=IfCondition(db_config),  # 条件：仅当db_config为True时启动
    )

    # 返回完整的启动描述，包含所有要启动的节点和配置
    return LaunchDescription(
        [
            tutorial_arg,  # 添加教程模式参数声明
            db_arg,  # 添加数据库参数声明
            ros2_control_hardware_type,  # 添加硬件类型参数声明
            rviz_node,  # 添加普通RViz节点
            rviz_node_tutorial,  # 添加教程RViz节点
            static_tf_node,  # 添加静态TF发布节点
            robot_state_publisher,  # 添加机器人状态发布节点
            move_group_node,  # 添加MoveIt核心节点
            ros2_control_node,  # 添加ROS 2控制节点
            joint_state_broadcaster_spawner,  # 添加关节状态广播器
            panda_arm_controller_spawner,  # 添加机械臂控制器
            panda_hand_controller_spawner,  # 添加手爪控制器
            mongodb_server_node,  # 添加数据库服务节点
        ]
    )