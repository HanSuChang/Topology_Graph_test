from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_transport",
                executable="republish",
                name="picam_compressed_republisher",
                arguments=["compressed", "raw"],
                remappings=[
                    ("in/compressed", "/picam/image_rotated/compressed"),
                    ("out", "/picam/image_rotated_view"),
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_image_view", "rqt_image_view"],
                output="screen",
            ),
        ]
    )
