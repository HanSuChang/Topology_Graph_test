from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    topology_file = LaunchConfiguration("topology_file")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    map_topic = LaunchConfiguration("map_topic")

    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("amr_topology"),
                    "launch",
                    "aruco_camera_detector.launch.py",
                ]
            )
        )
    )

    mission_loop = Node(
        package="amr_topology",
        executable="mission_loop",
        name="mission_loop",
        output="screen",
        parameters=[
            {
                "topology_file": topology_file,
                "map_frame": map_frame,
                "base_frame": base_frame,
                "cmd_vel_topic": cmd_vel_topic,
                "scan_topic": scan_topic,
                "map_topic": map_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "topology_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("amr_topology"),
                        "config",
                        "topology.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            aruco_launch,
            mission_loop,
        ]
    )
