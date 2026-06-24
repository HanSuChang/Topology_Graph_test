from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    rotation_degrees = LaunchConfiguration("rotation_degrees")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/picam/image_raw"),
            DeclareLaunchArgument("output_topic", default_value="/picam/image_rotated"),
            DeclareLaunchArgument("rotation_degrees", default_value="90"),
            Node(
                package="amr_topology",
                executable="image_rotate_node",
                name="picam_rotate_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "rotation_degrees": ParameterValue(rotation_degrees, value_type=int),
                    }
                ],
            ),
        ]
    )
