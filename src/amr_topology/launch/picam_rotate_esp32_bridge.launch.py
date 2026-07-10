from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    rotation_degrees = LaunchConfiguration("rotation_degrees")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    output_width = LaunchConfiguration("output_width")
    output_height = LaunchConfiguration("output_height")
    resize_mode = LaunchConfiguration("resize_mode")

    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    angle_topic = LaunchConfiguration("angle_topic")
    status_topic = LaunchConfiguration("status_topic")
    wait_for_ok = LaunchConfiguration("wait_for_ok")
    send_period_sec = LaunchConfiguration("send_period_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/picam/image_raw"),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/picam/image_rotated/compressed",
            ),
            DeclareLaunchArgument("rotation_degrees", default_value="90"),
            DeclareLaunchArgument("jpeg_quality", default_value="70"),
            DeclareLaunchArgument("output_width", default_value="0"),
            DeclareLaunchArgument("output_height", default_value="0"),
            DeclareLaunchArgument("resize_mode", default_value="none"),
            DeclareLaunchArgument("serial_port", default_value="auto"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("angle_topic", default_value="/arm_servo_angles"),
            DeclareLaunchArgument("status_topic", default_value="/arm_servo_status"),
            DeclareLaunchArgument("wait_for_ok", default_value="false"),
            DeclareLaunchArgument("send_period_sec", default_value="0.02"),
            Node(
                package="amr_topology",
                executable="image_rotate_compress_node",
                name="picam_rotate_compress_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "rotation_degrees": ParameterValue(
                            rotation_degrees,
                            value_type=int,
                        ),
                        "jpeg_quality": ParameterValue(jpeg_quality, value_type=int),
                        "output_width": ParameterValue(output_width, value_type=int),
                        "output_height": ParameterValue(output_height, value_type=int),
                        "resize_mode": resize_mode,
                    }
                ],
            ),
            Node(
                package="amr_topology",
                executable="esp32_servo_bridge_node.py",
                name="esp32_servo_bridge_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baudrate": ParameterValue(baudrate, value_type=int),
                        "angle_topic": angle_topic,
                        "status_topic": status_topic,
                        "wait_for_ok": ParameterValue(wait_for_ok, value_type=bool),
                        "send_period_sec": ParameterValue(
                            send_period_sec,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
