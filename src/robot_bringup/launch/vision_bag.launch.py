import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration('robot_type').perform(context)
    bag_path = LaunchConfiguration('bag_path').perform(context)
    
    bringup_dir = get_package_share_directory('robot_bringup')
    params_path = os.path.join(bringup_dir, 'config', robot_type, 'params.yaml')

    # 1. 确保 Iceoryx RouDi 运行
    start_roudi = ExecuteProcess(
        cmd=[os.path.join(bringup_dir, 'launch', 'start_roudi.sh')],
        output='screen'
    )

    # 2. 视觉算法容器 (绑核 0,1,2,3) - 移除了 hik_camera
    vision_container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        prefix=['nice -n -20 taskset -c 0,1,2,3'], 
        composable_node_descriptions=[
            ComposableNode(
                package='robot_auto_aim',
                plugin='robot_auto_aim::ArmorDetectorNode',
                name='armor_detector',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_auto_aim',
                plugin='robot_auto_aim::ArmorSolverNode',
                name='armor_solver',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='robot_ballistics',
                plugin='robot_ballistics::BallisticsNode',
                name='ballistics_node',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='both',
        respawn=True,
    )

    # 3. 运行 Rosbag 播放进程 (绑核 4,5)
    # 使用最高优先级，确保 250Hz 的发包不会因为系统调度而抖动
    play_bag = ExecuteProcess(
        cmd=['nice', '-n', '-20', 'taskset', '-c', '4,5', 'ros2', 'bag', 'play', '-l', bag_path],
        output='screen'
    )

    return [start_roudi, vision_container, play_bag]

def generate_launch_description():
    env_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', 
        '[{date_time_with_ms}] [{severity}] [{name}]: {message}'
    )
    env_color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    robot_type_arg = DeclareLaunchArgument('robot_type', default_value='default')
    # 新增 bag_path 参数，必须在启动时提供
    bag_path_arg = DeclareLaunchArgument('bag_path', description='Absolute path to the rosbag directory')
    
    return LaunchDescription([
        env_format,
        env_color,
        robot_type_arg,
        bag_path_arg,
        OpaqueFunction(function=launch_setup)
    ])
