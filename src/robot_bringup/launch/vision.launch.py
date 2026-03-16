import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # ---------------------------------------------------------
    # Arguments
    # ---------------------------------------------------------
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='default',
        description='Type of the robot (e.g., sentry, infantry_3, infantry_4, hero)'
    )

    # ---------------------------------------------------------
    # Paths
    # ---------------------------------------------------------
    bringup_dir = get_package_share_directory('robot_bringup')
    
    # Path to parameter file based on robot_type
    params_file = LaunchConfiguration('robot_type', default='default')
    params_path = [os.path.join(bringup_dir, 'config'), '/', params_file, '/params.yaml']

    # ---------------------------------------------------------
    # Vision Pipeline Container (Intra-process / Zero-copy)
    # ---------------------------------------------------------
    vision_container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Camera Driver
            ComposableNode(
                package='hik_camera_driver',
                plugin='hik_camera::HikCameraNode',
                name='hik_camera',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Armor Detector
            ComposableNode(
                package='robot_auto_aim',
                plugin='robot_auto_aim::ArmorDetectorNode',
                name='armor_detector',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Armor Solver
            ComposableNode(
                package='robot_auto_aim',
                plugin='robot_auto_aim::ArmorSolverNode',
                name='armor_solver',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Ballistics Node
            ComposableNode(
                package='robot_ballistics',
                plugin='robot_ballistics::BallisticsNode',
                name='ballistics_node',
                parameters=[params_path],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='both',
        # Auto-restart the whole container if it crashes
        respawn=True,
        respawn_delay=2.0,
    )

    # ---------------------------------------------------------
    # Standalone Nodes (Management / Communication)
    # ---------------------------------------------------------
    manager_node = Node(
        package='robot_manager',
        executable='manager_component_node',
        name='robot_manager',
        parameters=[params_path],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # ---------------------------------------------------------
    # Environment Setup for Zero-Copy (FastDDS / CycloneDDS)
    # ---------------------------------------------------------
    # Note: Inter-process zero-copy usually requires specific RMW configuration.
    # We enable intra-process here, which is standard for high-bandwidth image data in a container.
    
    return LaunchDescription([
        robot_type_arg,
        vision_container,
        manager_node
    ])
