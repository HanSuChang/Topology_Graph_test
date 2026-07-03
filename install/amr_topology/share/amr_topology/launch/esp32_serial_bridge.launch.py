from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    angle_topic = LaunchConfiguration("angle_topic")
    status_topic = LaunchConfiguration("status_topic")
    wait_for_ok = LaunchConfiguration("wait_for_ok")
    send_period_sec = LaunchConfiguration("send_period_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="auto"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("angle_topic", default_value="/arm_servo_angles"),
            DeclareLaunchArgument("status_topic", default_value="/arm_servo_status"),
            DeclareLaunchArgument("wait_for_ok", default_value="false"),
            DeclareLaunchArgument("send_period_sec", default_value="0.02"),
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
                        "send_period_sec": ParameterValue(send_period_sec, value_type=float),
                    }
                ],
            ),
        ]
    )
