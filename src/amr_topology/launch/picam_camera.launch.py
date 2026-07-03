from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera = LaunchConfiguration("camera")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    image_format = LaunchConfiguration("format")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera",
                default_value="/base/soc/i2c0mux/i2c@1/imx219@10",
            ),
            DeclareLaunchArgument("width", default_value="720"),
            DeclareLaunchArgument("height", default_value="1280"),
            DeclareLaunchArgument("format", default_value="RGB888"),
            DeclareLaunchArgument("image_topic", default_value="/picam/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/picam/camera_info"),
            Node(
                package="camera_ros",
                executable="camera_node",
                name="picam_camera",
                output="screen",
                parameters=[
                    {
                        "camera": camera,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "format": image_format,
                    }
                ],
                remappings=[
                    ("/camera/image_raw", image_topic),
                    ("/camera/camera_info", camera_info_topic),
                    ("image_raw", image_topic),
                    ("camera_info", camera_info_topic),
                ],
            ),
        ]
    )
