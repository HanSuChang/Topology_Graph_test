from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params')

    default_params = PathJoinSubstitution([
        FindPackageShare('amr_topology'),
        'config',
        'mppi_controller.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=default_params,
            description='MPPI rescue controller parameters',
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mppi_controller',
            output='screen',
            parameters=[params],
        ),
    ])
