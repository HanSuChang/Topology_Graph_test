from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration("video_device")
    image_topic = LaunchConfiguration("image_topic")
    pixel_format = LaunchConfiguration("pixel_format")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/v4l/by-id/usb-BY-5640-200408_Webcam_01.00.00-video-index0",
            ),
            DeclareLaunchArgument("image_topic", default_value="/aruco_webcam/image_raw"),
            DeclareLaunchArgument("pixel_format", default_value="YUYV"),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="aruco_webcam_check",
                output="screen",
                parameters=[
                    {
                        "video_device": video_device,
                        "pixel_format": pixel_format,
                    }
                ],
                remappings=[
                    ("image_raw", image_topic),
                ],
            ),
        ]
    )
