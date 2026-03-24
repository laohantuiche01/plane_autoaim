import os
import subprocess
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def get_default_robot_type():
    """自动检测 robot_type（优先级：环境变量 > 主机名识别）"""
    # 优先级 1：环境变量 ROBOT_TYPE
    if 'ROBOT_TYPE' in os.environ:
        return os.environ['ROBOT_TYPE']

    # 优先级 2：从主机名识别（脚本方式）
    try:
        bringup_dir = get_package_share_directory('robot_bringup')
        script_path = os.path.join(os.path.dirname(bringup_dir), '..', '..', 'tools', 'set_robot_type.sh')
        result = subprocess.run(['bash', script_path, '--get'],
                              capture_output=True, text=True, timeout=2)
        if result.returncode == 0:
            robot_type = result.stdout.strip()
            # 验证是否是有效的机型
            valid_types = ['default', 'infantry_3', 'infantry_4', 'sentry', 'outpost']
            if robot_type in valid_types:
                return robot_type
            # 如果识别出的主机名不在有效列表中，尝试映射
            hostname_to_type = {
                'infantry-3': 'infantry_3',
                'infantry-4': 'infantry_4',
                'sentry': 'sentry',
                'outpost': 'outpost',
            }
            for key, value in hostname_to_type.items():
                if key in robot_type.lower():
                    return value
    except Exception:
        pass

    # 默认值
    return 'default'

def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration('robot_type').perform(context)
    bag_path = LaunchConfiguration('bag_path').perform(context)
    
    bringup_dir = get_package_share_directory('robot_bringup')
    params_path = os.path.join(bringup_dir, 'config', robot_type, 'params.yaml')

    from launch.actions import TimerAction

    # ...

    # 1. 确保 Iceoryx RouDi 运行
    start_roudi = ExecuteProcess(
        cmd=[os.path.join(bringup_dir, 'launch', 'start_roudi.sh')],
        output='screen'
    )

    # 2. 视觉算法容器 (绑核 0,1,2,3) - 延迟 2 秒启动以等待 RouDi 就绪
    vision_container = TimerAction(
        period=2.0,
        actions=[
            ComposableNodeContainer(
                name='vision_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container_isolated',
                prefix=['chrt -f 90 taskset -c 0,1,2,3'], 
                composable_node_descriptions=[
                    ComposableNode(
                        package='robot_auto_aim',
                        plugin='robot_auto_aim::ArmorDetectorNode',
                        name='armor_detector',
                        parameters=[params_path, {'use_sim_time': True}],
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                    ComposableNode(
                        package='robot_auto_aim',
                        plugin='robot_auto_aim::ArmorSolverNode',
                        name='armor_solver',
                        parameters=[params_path, {'use_sim_time': True}],
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                    ComposableNode(
                        package='robot_ballistics',
                        plugin='robot_ballistics::BallisticsNode',
                        name='ballistics_node',
                        parameters=[params_path, {'use_sim_time': True}],
                        extra_arguments=[{'use_intra_process_comms': True}]
                    ),
                ],
                output='both',
                respawn=True,
            )
        ]
    )

    # 3. 运行 Rosbag 播放进程 (绑核 4,5) - 延迟 3 秒启动以等待算法节点就绪
    play_bag = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['nice', '-n', '-20', 'taskset', '-c', '4,5', 'ros2', 'bag', 'play', '--clock', '-l', bag_path],
                output='screen'
            )
        ]
    )

    # 4. 运行 Robot Manager 节点以自动激活生命周期
    from launch_ros.actions import Node
    manager_node = TimerAction(
        period=4.0, # 确保在容器之后启动
        actions=[
            Node(
                package='robot_manager',
                executable='manager_component_node',
                name='robot_manager',
                parameters=[params_path, {'use_sim_time': True}],
                output='both',
                respawn=True,
            )
        ]
    )

    return [start_roudi, vision_container, play_bag, manager_node]

def generate_launch_description():
    env_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', 
        '[{date_time_with_ms}] [{severity}] [{name}]: {message}'
    )
    env_color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=get_default_robot_type(),
        description='Type of the robot (default, sentry, infantry_3, infantry_4). Auto-detected from ROBOT_TYPE env or hostname if not specified.'
    )
    # 新增 bag_path 参数，必须在启动时提供
    bag_path_arg = DeclareLaunchArgument('bag_path', description='Absolute path to the rosbag directory')
    
    return LaunchDescription([
        env_format,
        env_color,
        robot_type_arg,
        bag_path_arg,
        OpaqueFunction(function=launch_setup)
    ])
