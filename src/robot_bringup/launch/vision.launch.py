import os
import subprocess
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
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
            valid_types = ['default', 'infantry_3', 'infantry_4', 'sentry', 'outpost', 'plane']
            if robot_type in valid_types:
                return robot_type
            # 如果识别出的主机名不在有效列表中，尝试映射
            hostname_to_type = {
                'infantry-3': 'infantry_3',
                'infantry-4': 'infantry_4',
                'sentry': 'sentry',
                'outpost': 'outpost',
                'plane': 'plane'
            }
            for key, value in hostname_to_type.items():
                if key in robot_type.lower():
                    return value
    except Exception:
        pass

    # 默认值
    return 'default'

def launch_setup(context, *args, **kwargs):
    # ---------------------------------------------------------
    # Configurations
    # ---------------------------------------------------------
    robot_type = LaunchConfiguration('robot_type').perform(context)
    use_file_log = LaunchConfiguration('use_file_log').perform(context).lower() == 'true'
    log_path_base = LaunchConfiguration('log_path').perform(context)
    
    bringup_dir = get_package_share_directory('robot_bringup')
    params_path = os.path.join(bringup_dir, 'config', robot_type, 'params.yaml')

    # ---------------------------------------------------------
    # Start Iceoryx RouDi Daemon (if not running)
    # ---------------------------------------------------------
    start_roudi = ExecuteProcess(
        cmd=[os.path.join(bringup_dir, 'launch', 'start_roudi.sh')],
        output='screen'
    )

    # Setup log directory with timestamp if file logging is enabled
    ros_log_dir = os.environ.get('ROS_LOG_DIR', os.path.expanduser('~/.ros/log'))
    if use_file_log:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        full_log_path = os.path.join(os.path.expanduser(log_path_base), timestamp)
        if not os.path.exists(full_log_path):
            os.makedirs(full_log_path, exist_ok=True)
        # Point ROS 2 logging to this directory
        os.environ['ROS_LOG_DIR'] = full_log_path
        print(f"Logging to: {full_log_path}")

    # ---------------------------------------------------------
    # Vision Pipeline Container (Intra-process / Zero-copy)
    # ---------------------------------------------------------
    vision_container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        prefix=['chrt -f 90 taskset -c 22,23,24,25,26,27,28,29,30,31'], # SCHED_FIFO priority 90 and CPU binding
        composable_node_descriptions=[
            ComposableNode(
                package='hik_camera_driver',
                plugin='HikCameraDriver',
                name='hik_camera',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
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
        respawn_delay=2.0,
    )

    # ---------------------------------------------------------
    # Standalone Nodes
    # ---------------------------------------------------------
    manager_node = Node(
        package='robot_manager',
        executable='manager_component_node',
        name='robot_manager',
        parameters=[params_path],
        output='both',
        respawn=True,
        respawn_delay=2.0,
    )

    communication_node = Node(
        package='robot_communication',
        executable='robot_communication_node',
        name='robot_communication',
        parameters=[params_path],
        output='both',
        respawn=True,
        respawn_delay=2.0,
    )

    return [start_roudi, vision_container, manager_node, communication_node]

def generate_launch_description():
    # Environment Variables for detailed logging
    env_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', 
        '[{date_time_with_ms}] [{severity}] [{name}] [{file_name}:{line_number} ({function_name})]: {message}'
    )
    env_color = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    # Arguments
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=get_default_robot_type(),
        description='Type of the robot (default, sentry, infantry_3, infantry_4). Auto-detected from ROBOT_TYPE env or hostname if not specified.'
    )
    use_file_log_arg = DeclareLaunchArgument(
        'use_file_log',
        default_value='true',
        description='Whether to save logs to file'
    )
    log_path_arg = DeclareLaunchArgument(
        'log_path',
        default_value='~/ckyf_vision_log',
        description='Base directory for logs'
    )
    
    return LaunchDescription([
        env_format,
        env_color,
        robot_type_arg,
        use_file_log_arg,
        log_path_arg,
        OpaqueFunction(function=launch_setup)
    ])
