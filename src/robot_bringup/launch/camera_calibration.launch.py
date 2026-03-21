import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    robot_type = LaunchConfiguration('robot_type').perform(context)
    size = LaunchConfiguration('size').perform(context)
    square = LaunchConfiguration('square').perform(context)
    exposure_time = LaunchConfiguration('exposure_time').perform(context)
    
    bringup_dir = get_package_share_directory('robot_bringup')
    params_path = os.path.join(bringup_dir, 'config', robot_type, 'params.yaml')

    # Camera driver node
    hik_camera_node = Node(
        package='hik_camera_driver',
        executable='hik_camera_driver_node',
        name='hik_camera',
        parameters=[
            params_path,
            {'exposure_time': float(exposure_time)}
        ],
        output='screen'
    )

    # Calibration node
    calibrator_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        name='cameracalibrator',
        arguments=[
            '--size', size,
            '--square', square,
            '--no-service-check'
        ],
        remappings=[
            ('image', '/image_raw'),
            ('camera', '/')
        ],
        output='screen'
    )

    return [hik_camera_node, calibrator_node]

def generate_launch_description():
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='default',
        description='Type of the robot (default, sentry, infantry_3, infantry_4)'
    )
    
    size_arg = DeclareLaunchArgument(
        'size',
        default_value='11x8',
        description='Chessboard size as NxM, counting interior corners (e.g. 11x8)'
    )
    
    square_arg = DeclareLaunchArgument(
        'square',
        default_value='0.015',
        description='Chessboard square size in meters (e.g. 0.015 for 15mm)'
    )
    
    exposure_time_arg = DeclareLaunchArgument(
        'exposure_time',
        default_value='5000.0',
        description='Camera exposure time in microseconds'
    )

    return LaunchDescription([
        robot_type_arg,
        size_arg,
        square_arg,
        exposure_time_arg,
        OpaqueFunction(function=launch_setup)
    ])
