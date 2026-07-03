from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    target_color = LaunchConfiguration("target_color")
    camera_backend = LaunchConfiguration("camera_backend")
    camera_index = LaunchConfiguration("camera_index")
    camera_device = LaunchConfiguration("camera_device")
    image_topic = LaunchConfiguration("image_topic")
    compressed_image_topic = LaunchConfiguration("compressed_image_topic")
    enable_picam_rotate = LaunchConfiguration("enable_picam_rotate")
    picam_input_topic = LaunchConfiguration("picam_input_topic")
    rotation_degrees = LaunchConfiguration("rotation_degrees")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    auto_start_enabled = LaunchConfiguration("auto_start_enabled")
    auto_start_delay_sec = LaunchConfiguration("auto_start_delay_sec")
    distance_correction_scale = LaunchConfiguration("distance_correction_scale")
    approach_done_distance_mm = LaunchConfiguration("approach_done_distance_mm")
    enable_serial_bridge = LaunchConfiguration("enable_serial_bridge")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    serial_port = LaunchConfiguration("serial_port")
    display = LaunchConfiguration("display")

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_color", default_value="blue"),
            DeclareLaunchArgument("camera_backend", default_value="ros_compressed"),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("camera_device", default_value=""),
            DeclareLaunchArgument("image_topic", default_value="/picam/image_raw"),
            DeclareLaunchArgument("compressed_image_topic", default_value="/picam/image_rotated/compressed"),
            DeclareLaunchArgument("enable_picam_rotate", default_value="false"),
            DeclareLaunchArgument("picam_input_topic", default_value="/picam/image_raw"),
            DeclareLaunchArgument("rotation_degrees", default_value="90"),
            DeclareLaunchArgument("jpeg_quality", default_value="70"),
            DeclareLaunchArgument("auto_start_enabled", default_value="true"),
            DeclareLaunchArgument("auto_start_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("distance_correction_scale", default_value="2.17647"),
            DeclareLaunchArgument("approach_done_distance_mm", default_value="50.0"),
            DeclareLaunchArgument("enable_serial_bridge", default_value="false"),
            DeclareLaunchArgument("enable_rosbridge", default_value="false"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("serial_port", default_value="auto"),
            DeclareLaunchArgument("display", default_value="true"),
            Node(
                package="amr_topology",
                executable="image_rotate_compress_node",
                name="picam_rotate_compress_node",
                output="screen",
                condition=IfCondition(enable_picam_rotate),
                parameters=[
                    {
                        "input_topic": picam_input_topic,
                        "output_topic": compressed_image_topic,
                        "rotation_degrees": ParameterValue(rotation_degrees, value_type=int),
                        "jpeg_quality": ParameterValue(jpeg_quality, value_type=int),
                    }
                ],
            ),
            Node(
                package="amr_topology",
                executable="esp32_servo_bridge_node.py",
                name="esp32_servo_bridge_node",
                output="screen",
                condition=IfCondition(enable_serial_bridge),
                parameters=[
                    {
                        "serial_port": serial_port,
                        "angle_topic": "/arm_servo_angles",
                        "status_topic": "/arm_servo_status",
                        "wait_for_ok": True,
                    }
                ],
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                condition=IfCondition(enable_rosbridge),
                parameters=[
                    {
                        "port": ParameterValue(rosbridge_port, value_type=int),
                    }
                ],
            ),
            Node(
                package="amr_topology",
                executable="arm_vision_gripper_node.py",
                name="arm_vision_gripper_node",
                output="screen",
                parameters=[
                    {
                        "target_color": target_color,
                        "camera_backend": camera_backend,
                        "camera_index": ParameterValue(camera_index, value_type=int),
                        "camera_device": camera_device,
                        "image_topic": image_topic,
                        "compressed_image_topic": compressed_image_topic,
                        "display": ParameterValue(display, value_type=bool),
                        "auto_start_enabled": ParameterValue(
                            auto_start_enabled,
                            value_type=bool,
                        ),
                        "auto_start_delay_sec": ParameterValue(
                            auto_start_delay_sec,
                            value_type=float,
                        ),
                        "distance_correction_scale": ParameterValue(
                            distance_correction_scale,
                            value_type=float,
                        ),
                        "approach_done_distance_mm": ParameterValue(
                            approach_done_distance_mm,
                            value_type=float,
                        ),
                        "servo_angle_topic": "/arm_servo_angles",
                        "status_topic": "/arm_vision_status",
                        "command_topic": "/arm_vision_command",
                    }
                ],
            ),
        ]
    )
