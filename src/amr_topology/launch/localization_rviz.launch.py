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
    rc_a_stop_x_arg = DeclareLaunchArgument(
        'rc_a_stop_x',
        default_value='0.7064711451530457',
        description='RC car A slot stop marker x coordinate in map frame',
    )
    rc_a_stop_y_arg = DeclareLaunchArgument(
        'rc_a_stop_y',
        default_value='0.29016929864883423',
        description='RC car A slot stop marker y coordinate in map frame',
    )
    rc_b_stop_x_arg = DeclareLaunchArgument(
        'rc_b_stop_x',
        default_value='0.7269076704978943',
        description='RC car B slot stop marker x coordinate in map frame',
    )
    rc_b_stop_y_arg = DeclareLaunchArgument(
        'rc_b_stop_y',
        default_value='-1.4004881381988525',
        description='RC car B slot stop marker y coordinate in map frame',
    )
    marker_radius_arg = DeclareLaunchArgument(
        'marker_radius',
        default_value='0.08',
        description='Topology marker circle diameter in meters',
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

    topology_markers = Node(
        package='amr_topology',
        executable='topology_marker_node',
        name='topology_marker_node',
        output='screen',
        parameters=[{
            'topology_file': PathJoinSubstitution([
                FindPackageShare('amr_topology'),
                'config',
                'topology.yaml',
            ]),
            'frame_id': 'map',
            'rc_a_stop_x': LaunchConfiguration('rc_a_stop_x'),
            'rc_a_stop_y': LaunchConfiguration('rc_a_stop_y'),
            'rc_b_stop_x': LaunchConfiguration('rc_b_stop_x'),
            'rc_b_stop_y': LaunchConfiguration('rc_b_stop_y'),
            'marker_radius': LaunchConfiguration('marker_radius'),
        }],
    )

    return LaunchDescription([
        map_arg,
        amcl_params_arg,
        rviz_config_arg,
        rc_a_stop_x_arg,
        rc_a_stop_y_arg,
        rc_b_stop_x_arg,
        rc_b_stop_y_arg,
        marker_radius_arg,
        map_server,
        amcl,
        lifecycle_manager,
        topology_markers,
        rviz,
    ])
