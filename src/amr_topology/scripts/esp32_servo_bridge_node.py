#!/usr/bin/env python3

import glob
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String

try:
    import serial
except ImportError:  # pragma: no cover - reported at runtime on the robot
    serial = None


SERVO_COUNT = 5


class Esp32ServoBridge(Node):
    def __init__(self):
        super().__init__("esp32_servo_bridge_node")

        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("angle_topic", "/arm_servo_angles")
        self.declare_parameter("status_topic", "/arm_servo_status")
        self.declare_parameter("connect_retry_sec", 2.0)
        self.declare_parameter("send_period_sec", 0.05)
        self.declare_parameter("ack_timeout_sec", 8.0)
        self.declare_parameter("wait_for_ok", True)
        self.declare_parameter("send_duplicates", False)

        self.serial_port_param = self.get_parameter("serial_port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.connect_retry_sec = float(self.get_parameter("connect_retry_sec").value)
        self.ack_timeout_sec = float(self.get_parameter("ack_timeout_sec").value)
        self.wait_for_ok = bool(self.get_parameter("wait_for_ok").value)
        self.send_duplicates = bool(self.get_parameter("send_duplicates").value)

        self.ser = None
        self.pending_angles = None
        self.last_sent_angles = None
        self.last_connect_attempt = 0.0

        angle_topic = self.get_parameter("angle_topic").value
        status_topic = self.get_parameter("status_topic").value

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Int32MultiArray, angle_topic, self.handle_angles, 10)
        self.timer = self.create_timer(
            float(self.get_parameter("send_period_sec").value),
            self.timer_step,
        )

        self.publish_status(f"waiting for ESP32 serial: {self.serial_port_param}")

    def handle_angles(self, msg):
        if len(msg.data) != SERVO_COUNT:
            self.get_logger().warn(
                f"ignored servo command with {len(msg.data)} values; expected {SERVO_COUNT}"
            )
            return

        angles = [int(value) for value in msg.data]
        invalid = [value for value in angles if value < 0 or value > 180]
        if invalid:
            self.get_logger().warn(f"ignored out-of-range servo command: {angles}")
            return

        self.pending_angles = angles

    def timer_step(self):
        if self.ser is None or not self.ser.is_open:
            self.try_connect()
            return

        if self.pending_angles is None:
            return

        angles = self.pending_angles
        if not self.send_duplicates and angles == self.last_sent_angles:
            self.pending_angles = None
            return

        if self.write_angles(angles):
            self.last_sent_angles = angles
            self.pending_angles = None

    def try_connect(self):
        now = time.monotonic()
        if now - self.last_connect_attempt < self.connect_retry_sec:
            return

        self.last_connect_attempt = now

        if serial is None:
            self.publish_status("pyserial is not installed; install python3-serial")
            return

        for port in self.serial_candidates():
            try:
                self.ser = serial.Serial(port, self.baudrate, timeout=0.05)
                time.sleep(2.0)
                self.ser.reset_input_buffer()
                self.publish_status(f"connected {port} @ {self.baudrate}")
                self.get_logger().info(f"ESP32 serial connected: {port} @ {self.baudrate}")
                return
            except serial.SerialException as exc:
                self.get_logger().debug(f"failed to open {port}: {exc}")

        self.publish_status(f"no ESP32 serial port found: {self.serial_candidates()}")

    def serial_candidates(self):
        if self.serial_port_param != "auto":
            return [self.serial_port_param]

        by_id = sorted(glob.glob("/dev/serial/by-id/*"))
        usb = sorted(glob.glob("/dev/ttyUSB*"))
        acm = sorted(glob.glob("/dev/ttyACM*"))

        preferred_by_id = [
            port for port in by_id
            if "OpenCR" not in port and "ROBOTIS" not in port
        ]
        fallback_by_id = [
            port for port in by_id
            if port not in preferred_by_id
        ]

        candidates = []
        for port in preferred_by_id + usb + fallback_by_id + acm:
            real_port = os.path.realpath(port)
            if port not in candidates:
                candidates.append(port)
            if real_port not in candidates:
                candidates.append(real_port)
        return candidates

    def write_angles(self, angles):
        command = ",".join(str(value) for value in angles) + "\n"

        try:
            self.ser.write(command.encode("utf-8"))
            self.ser.flush()
            self.get_logger().info(f"TX {command.strip()}")

            if self.wait_for_ok:
                return self.wait_for_ack(command.strip())

            return True
        except serial.SerialException as exc:
            self.publish_status(f"serial write failed: {exc}")
            self.close_serial()
            return False

    def wait_for_ack(self, command):
        deadline = time.monotonic() + self.ack_timeout_sec

        while time.monotonic() < deadline:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except serial.SerialException as exc:
                self.publish_status(f"serial read failed: {exc}")
                self.close_serial()
                return False

            if not line:
                continue

            self.get_logger().info(f"ESP32 {line}")
            self.publish_status(line)

            if line.startswith("OK"):
                return True
            if line.startswith("ERR"):
                return False

        self.publish_status(f"ack timeout after TX {command}")
        return False

    def close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = Esp32ServoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
