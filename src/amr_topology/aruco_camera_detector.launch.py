from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    video_device = LaunchConfiguration("video_device")
    pixel_format = LaunchConfiguration("pixel_format")
    dictionary = LaunchConfiguration("dictionary")
    marker_id = LaunchConfiguration("marker_id")
    marker_size_m = LaunchConfiguration("marker_size_m")
    focal_length_px = LaunchConfiguration("focal_length_px")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/v4l/by-id/usb-BY-5640-200408_Webcam_01.00.00-video-index0",
            ),
            DeclareLaunchArgument("pixel_format", default_value="YUYV"),
            DeclareLaunchArgument("dictionary", default_value="4X4"),
            DeclareLaunchArgument("marker_id", default_value="175"),
            DeclareLaunchArgument("marker_size_m", default_value="0.095"),
            DeclareLaunchArgument("focal_length_px", default_value="620.0"),
            DeclareLaunchArgument("camera_width", default_value="640"),
            DeclareLaunchArgument("camera_height", default_value="480"),
            DeclareLaunchArgument("camera_fps", default_value="10.0"),
            Node(
                package="amr_topology",
                executable="aruco_marker_detector_node",
                name="aruco_marker_detector_node",
                output="screen",
                parameters=[
                    {
                        "video_device": video_device,
                        "pixel_format": pixel_format,
                        "dictionary": dictionary,
                        "marker_id": ParameterValue(marker_id, value_type=int),
                        "marker_size_m": ParameterValue(marker_size_m, value_type=float),
                        "focal_length_px": ParameterValue(focal_length_px, value_type=float),
                        "camera_width": ParameterValue(camera_width, value_type=int),
                        "camera_height": ParameterValue(camera_height, value_type=int),
                        "camera_fps": ParameterValue(camera_fps, value_type=float),
                    }
                ],
            ),
        ]
    )
