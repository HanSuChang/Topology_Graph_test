from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    angle_topic = LaunchConfiguration("angle_topic")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    angles = LaunchConfiguration("angles")

    return LaunchDescription(
        [
            DeclareLaunchArgument("angle_topic", default_value="/arm_servo_angles"),
            DeclareLaunchArgument("publish_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("angles", default_value="90,90,90,90,90"),
            Node(
                package="amr_topology",
                executable="servo_hold_90_node.py",
                name="servo_hold_90_node",
                output="screen",
                parameters=[
                    {
                        "angle_topic": angle_topic,
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "angles": angles,
                    }
                ],
            ),
        ]
    )
