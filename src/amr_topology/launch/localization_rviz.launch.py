from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_map = PathJoinSubstitution([
        FindPackageShare('amr_topology'),
        'maps',
        'map.yaml',
    ])
    default_amcl_params = PathJoinSubstitution([
        FindPackageShare('amr_topology'),
        'config',
        'amcl.yaml',
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('amr_topology'),
        'rviz',
        'amcl_localization.rviz',
    ])

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file',
    )
    amcl_params_arg = DeclareLaunchArgument(
        'amcl_params',
        default_value=default_amcl_params,
        description='Full path to the AMCL params yaml file',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz config file',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[LaunchConfiguration('amcl_params')],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        map_arg,
        amcl_params_arg,
        rviz_config_arg,
        map_server,
        amcl,
        lifecycle_manager,
        rviz,
    ])
