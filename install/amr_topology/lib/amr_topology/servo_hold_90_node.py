#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class ServoHold90Node(Node):
    def __init__(self):
        super().__init__("servo_hold_90_node")

        self.declare_parameter("angle_topic", "/arm_servo_angles")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("angles", [90, 90, 90, 90, 90])

        angle_topic = self.get_parameter("angle_topic").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.angles = [int(value) for value in self.get_parameter("angles").value]

        self.publisher = self.create_publisher(Int32MultiArray, angle_topic, 10)
        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 0.1), self.publish_angles)

        self.get_logger().info(
            f"publishing servo hold angles {self.angles} to {angle_topic} at {publish_rate_hz:.2f} Hz"
        )

    def publish_angles(self):
        msg = Int32MultiArray()
        msg.data = self.angles
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ServoHold90Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
