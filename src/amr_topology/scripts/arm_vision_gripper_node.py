#!/usr/bin/env python3

import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String


SERVO_COUNT = 5


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class CameraSource:
    def __init__(self, node):
        self.node = node
        self.backend = node.get_parameter("camera_backend").value
        self.camera_index = int(node.get_parameter("camera_index").value)
        self.camera_device = node.get_parameter("camera_device").value
        self.width = int(node.get_parameter("camera_width").value)
        self.height = int(node.get_parameter("camera_height").value)
        self.fps = float(node.get_parameter("camera_fps").value)
        self.picam2 = None
        self.cap = None

    def open(self):
        if self.backend in ("auto", "picamera2") and self.open_picamera2():
            return True

        if self.backend in ("auto", "v4l2", "opencv") and self.open_v4l2():
            return True

        return False

    def open_picamera2(self):
        try:
            from picamera2 import Picamera2
        except ImportError:
            if self.backend == "picamera2":
                self.node.get_logger().error("picamera2 is not installed")
            return False

        try:
            self.picam2 = Picamera2()
            config = self.picam2.create_preview_configuration(
                main={"format": "BGR888", "size": (self.width, self.height)}
            )
            self.picam2.configure(config)
            self.picam2.start()
            self.node.get_logger().info("camera opened with Picamera2")
            return True
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.node.get_logger().warn(f"failed to open Picamera2 camera: {exc}")
            self.picam2 = None
            return False

    def open_v4l2(self):
        source = self.camera_device if self.camera_device else self.camera_index
        self.cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = None
            self.node.get_logger().error(f"failed to open V4L2 camera: {source}")
            return False

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.node.get_logger().info(f"camera opened with V4L2: {source}")
        return True

    def read(self):
        if self.picam2 is not None:
            frame = self.picam2.capture_array()
            return frame is not None, frame

        if self.cap is not None:
            return self.cap.read()

        return False, None

    def close(self):
        if self.picam2 is not None:
            self.picam2.stop()
            self.picam2 = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None


class ArmVisionGripper(Node):
    def __init__(self):
        super().__init__("arm_vision_gripper_node")

        self.declare_parameter("target_color", "blue")
        self.declare_parameter("camera_backend", "auto")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_device", "")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15.0)
        self.declare_parameter("image_topic", "/picam/image_raw")
        self.declare_parameter("compressed_image_topic", "/picam/image_rotated/compressed")
        self.declare_parameter("display", bool(os.environ.get("DISPLAY")))
        self.declare_parameter("servo_angle_topic", "/arm_servo_angles")
        self.declare_parameter("status_topic", "/arm_vision_status")
        self.declare_parameter("command_topic", "/arm_vision_command")
        self.declare_parameter("timer_period_sec", 0.03)
        self.declare_parameter("control_interval_sec", 0.35)
        self.declare_parameter("center_tolerance_u", 30.0)
        self.declare_parameter("center_tolerance_v", 30.0)
        self.declare_parameter("auto_align_hold_sec", 3.0)
        self.declare_parameter("cube_size_mm", 30.0)
        self.declare_parameter("focal_length_px", 700.0)
        self.declare_parameter("distance_correction_scale", 2.17647)
        self.declare_parameter("approach_done_distance_mm", 50.0)
        self.declare_parameter("target_object_r_mm", 280.0)
        self.declare_parameter("grip_target_z_mm", 15.0)
        self.declare_parameter("auto_start_enabled", True)
        self.declare_parameter("auto_start_delay_sec", 5.0)
        self.declare_parameter("servo_republish_interval_sec", 1.0)

        self.target_color = self.get_parameter("target_color").value
        self.display = bool(self.get_parameter("display").value)
        self.control_interval = float(self.get_parameter("control_interval_sec").value)
        self.center_tolerance_u = float(self.get_parameter("center_tolerance_u").value)
        self.center_tolerance_v = float(self.get_parameter("center_tolerance_v").value)
        self.auto_align_hold_sec = float(self.get_parameter("auto_align_hold_sec").value)
        self.cube_size_mm = float(self.get_parameter("cube_size_mm").value)
        self.focal_length_px = float(self.get_parameter("focal_length_px").value)
        self.distance_correction_scale = float(
            self.get_parameter("distance_correction_scale").value
        )
        self.approach_done_distance_mm = float(
            self.get_parameter("approach_done_distance_mm").value
        )
        self.target_object_r_mm = float(self.get_parameter("target_object_r_mm").value)
        self.grip_target_z_mm = float(self.get_parameter("grip_target_z_mm").value)
        self.auto_start_enabled = bool(self.get_parameter("auto_start_enabled").value)
        self.auto_start_delay_sec = float(self.get_parameter("auto_start_delay_sec").value)
        self.servo_republish_interval_sec = float(
            self.get_parameter("servo_republish_interval_sec").value
        )

        self.center_tolerance_u = float(self.get_parameter("center_tolerance_u").value)
        self.center_tolerance_v = float(self.get_parameter("center_tolerance_v").value)

        self.s0_step = 1.0
        self.v_step = 2.0
        self.s1_weight = 0.70
        self.s2_weight = 0.30
        self.v_direction = 1.0

        self.error_v_filter_alpha = 0.25
        self.filtered_error_v = 0.0

        self.d1_mm = 110.0
        self.d2_mm = 70.0
        self.d3_mm = 105.0
        self.base_to_link1_z_mm = 25.0
        self.link2_angle_at_90_deg = 30.25643716
        self.link3_offset_deg = 59.74356284
        self.ik_approach_r_step_mm = 0.6
        self.ik_approach_z_step_mm = 0.4
        self.ik_wrist_from_image_gain = 0.012
        self.ik_wrist_step_limit_deg = 0.7

        self.start_pose = [38.0, 70.0, 90.0, 15.0, 50.0]
        self.drop_pose = [130.0, 120.0, 80.0, 90.0, 50.0]
        self.gripper_open_angle = 140.0
        self.gripper_close_angle = 90.0
        self.insert_final_s1_offset_deg = 24.0
        self.insert_final_s2_offset_deg = -25.0
        self.insert_final_s3_offset_deg = 20.0
        self.gripper_open_wait_sec = 5.0
        self.insert_joint_step_wait_sec = 2.0
        self.gripper_close_before_wait_sec = 5.0
        self.return_to_start_delay_sec = 1.0

        self.s0, self.s1, self.s2, self.s3, self.gripper = self.start_pose
        self.auto_approach = False
        self.insert_sequence_done = False
        self.target_lock = None
        self.align_hold_start = None
        self.approach_released = False
        self.approach_stop_latched = False
        self.last_control_time = 0.0
        self.sequence_running = False
        self.latest_frame = None
        self.last_waiting_frame_log = 0.0
        self.object_tracks = {}
        self.next_track_number = 1
        self.track_match_max_distance_px = 180.0
        self.track_max_missed_frames = 8
        self.search_pose_enter_time = time.monotonic()
        self.last_servo_publish_time = 0.0

        servo_angle_topic = self.get_parameter("servo_angle_topic").value
        status_topic = self.get_parameter("status_topic").value
        command_topic = self.get_parameter("command_topic").value
        self.camera_backend = self.get_parameter("camera_backend").value

        self.servo_pub = self.create_publisher(Int32MultiArray, servo_angle_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, command_topic, self.handle_command, 10)

        self.camera = None
        self.configure_frame_source()

        self.publish_angles(self.current_pose())
        self.publish_status(
            f"ready: auto starts after {self.auto_start_delay_sec:.1f}s in search pose"
        )

        self.timer = self.create_timer(
            float(self.get_parameter("timer_period_sec").value),
            self.timer_step,
        )

    def configure_frame_source(self):
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        if self.camera_backend in ("ros_compressed", "compressed_topic"):
            topic = self.get_parameter("compressed_image_topic").value
            self.create_subscription(
                CompressedImage,
                topic,
                self.handle_compressed_image,
                image_qos,
            )
            self.get_logger().info(f"subscribing compressed image topic: {topic}")
            return

        if self.camera_backend in ("ros_image", "image_topic"):
            topic = self.get_parameter("image_topic").value
            self.create_subscription(Image, topic, self.handle_image, image_qos)
            self.get_logger().info(f"subscribing image topic: {topic}")
            return

        self.camera = CameraSource(self)
        if not self.camera.open():
            self.get_logger().error("camera open failed; node will stay idle")

    def handle_compressed_image(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if frame is not None:
            self.latest_frame = frame

    def handle_image(self, msg):
        frame = self.image_msg_to_frame(msg)
        if frame is not None:
            self.latest_frame = frame

    def image_msg_to_frame(self, msg):
        if msg.encoding not in ("bgr8", "rgb8", "mono8"):
            self.get_logger().warn(f"unsupported image encoding: {msg.encoding}")
            return None

        channels = 1 if msg.encoding == "mono8" else 3
        expected_step = msg.width * channels
        if msg.step < expected_step:
            self.get_logger().warn("invalid image step")
            return None

        array = np.frombuffer(msg.data, dtype=np.uint8)
        frame = array.reshape((msg.height, msg.step // channels, channels))
        frame = frame[:, : msg.width].copy()

        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "mono8":
            frame = frame[:, :, 0]
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        return frame

    def current_pose(self):
        return [self.s0, self.s1, self.s2, self.s3, self.gripper]

    def handle_command(self, msg):
        command = msg.data.strip().lower()
        if command in ("a", "start", "auto"):
            self.start_auto()
        elif command in ("stop", "off"):
            self.stop_auto()
        elif command in ("c", "reset"):
            self.reset_to_start()
        elif command in ("r", "red"):
            self.set_target_color("red")
        elif command in ("b", "blue"):
            self.set_target_color("blue")
        else:
            self.get_logger().warn(f"unknown arm vision command: {msg.data}")

    def timer_step(self):
        if self.sequence_running:
            return

        now = time.monotonic()
        self.republish_current_pose_if_needed(now)

        if self.camera is not None:
            ok, frame = self.camera.read()
            if not ok or frame is None:
                self.publish_status("camera frame read failed")
                return
        else:
            if self.latest_frame is None:
                if now - self.last_waiting_frame_log >= 2.0:
                    self.publish_status("waiting for camera topic frame")
                    self.last_waiting_frame_log = now
                return
            frame = self.latest_frame.copy()

        result, output, mask = self.detect_colored_cube(frame)

        if result is not None:
            self.process_detection(result, now, output)
        else:
            self.process_missing_detection(now, output)

        if self.display:
            cv2.imshow("arm_vision_camera", output)
            cv2.imshow("arm_vision_mask", mask)
            self.handle_key(cv2.waitKey(1) & 0xFF, result)

    def handle_key(self, key, result):
        if key == 255:
            return
        if key == ord("q"):
            rclpy.shutdown()
        elif key == ord("r"):
            self.set_target_color("red")
        elif key == ord("b"):
            self.set_target_color("blue")
        elif key == ord("c"):
            self.reset_to_start()
        elif key == ord("a"):
            if self.auto_approach:
                self.stop_auto()
            elif result is not None:
                self.start_auto(result)
            else:
                self.publish_status("auto not started: no target detected")

    def process_detection(self, result, now, output):
        error_u = result["error_u"]
        error_v = result["error_v"]
        distance_mm = result["distance_mm"]
        aligned_now_u = abs(error_u) <= self.center_tolerance_u
        aligned_now_v = abs(error_v) <= self.center_tolerance_v

        self.filtered_error_v = (
            self.error_v_filter_alpha * error_v
            + (1.0 - self.error_v_filter_alpha) * self.filtered_error_v
        )

        if self.should_auto_start(now):
            self.start_auto(result, reason="auto start after search-pose delay")

        if now - self.last_control_time < self.control_interval:
            return

        old_pose = self.current_pose()

        if self.insert_sequence_done:
            aligned_u = True
            aligned_v = True
        elif self.auto_approach:
            self.target_lock = (result["object_u"], result["object_v"])
            self.s0, aligned_u = self.update_s0_by_error(error_u, self.s0)

            if not self.approach_released:
                self.s1, self.s2, self.s3, aligned_v = self.update_s12_by_error(
                    error_v, self.s1, self.s2, self.s3
                )
                if aligned_u and aligned_v:
                    if self.align_hold_start is None:
                        self.align_hold_start = now
                    elif now - self.align_hold_start >= self.auto_align_hold_sec:
                        self.approach_released = True
                        self.publish_status("center held; starting approach")
                else:
                    self.align_hold_start = None
            else:
                aligned_v = aligned_now_v
        else:
            aligned_u = aligned_now_u
            aligned_v = aligned_now_v

        approach_done = (
            self.auto_approach
            and self.approach_released
            and distance_mm is not None
            and distance_mm <= self.approach_done_distance_mm
            and aligned_u
            and aligned_v
        )

        approach_status = None
        if self.auto_approach and self.approach_released and aligned_u and not approach_done:
            (
                self.s1,
                self.s2,
                self.s3,
                approach_done,
                approach_status,
            ) = self.update_approach_by_ik(
                self.filtered_error_v, self.s1, self.s2, self.s3
            )

        if approach_done and not self.approach_stop_latched:
            self.publish_status("approach done; holding before grip sequence")
            self.approach_stop_latched = True
            self.auto_approach = False
            self.target_lock = None
            self.align_hold_start = None
            self.approach_released = False
            self.last_control_time = now
            return

        if self.pose_changed(old_pose, self.current_pose()):
            self.publish_angles(self.current_pose())

        self.draw_state(output, aligned_u, aligned_v, approach_done, approach_status, now)
        self.last_control_time = now

    def process_missing_detection(self, now, output):
        if self.auto_approach:
            self.align_hold_start = None

        cv2.putText(
            output,
            "No target detected",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    def detect_colored_cube(self, frame):
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = self.get_color_mask(hsv)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center_u = width / 2.0
        center_v = height / 2.0
        candidates = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / float(h)
            if not 0.5 < aspect_ratio < 1.8:
                continue

            object_u = x + w / 2.0
            object_v = y + h / 3.0
            candidates.append(
                {
                    "object_u": object_u,
                    "object_v": object_v,
                    "bbox": (x, y, w, h),
                    "bbox_width": w,
                    "bbox_height": h,
                    "area": area,
                    "aspect_ratio": aspect_ratio,
                    "distance_mm": self.estimate_cube_distance_mm(w, h),
                }
            )

        candidates = self.suppress_inner_candidates(candidates)
        if not candidates:
            self.age_unmatched_tracks()
            return None, frame, mask

        self.assign_persistent_numbers(candidates)
        candidates.sort(key=lambda item: item["number"])

        if self.target_lock is None:
            selected = candidates[0]
        else:
            lock_u, lock_v = self.target_lock
            selected = min(
                candidates,
                key=lambda item: (item["object_u"] - lock_u) ** 2
                + (item["object_v"] - lock_v) ** 2,
            )

        self.draw_candidates(frame, candidates, selected)

        object_u = selected["object_u"]
        object_v = selected["object_v"]
        error_u = object_u - center_u
        error_v = object_v - center_v

        result = {
            "object_u": object_u,
            "object_v": object_v,
            "error_u": error_u,
            "error_v": error_v,
            "bbox": selected["bbox"],
            "bbox_width": selected["bbox_width"],
            "bbox_height": selected["bbox_height"],
            "area": selected["area"],
            "aspect_ratio": selected["aspect_ratio"],
            "distance_mm": selected["distance_mm"],
            "frame_width": width,
            "frame_height": height,
            "target_number": selected["number"],
            "detected_count": len(candidates),
            "target_locked": self.target_lock is not None,
        }

        cv2.circle(frame, (int(center_u), int(center_v)), 5, (255, 255, 255), -1)
        cv2.line(
            frame,
            (int(center_u), int(center_v)),
            (int(object_u), int(object_v)),
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            f"{self.target_color} cube #{selected['number']}/{len(candidates)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"err_u={error_u:.1f}, err_v={error_v:.1f}, dist={selected['distance_mm']:.0f}mm",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
        )
        return result, frame, mask

    def get_color_mask(self, hsv):
        if self.target_color == "red":
            lower_red1 = np.array([0, 80, 80])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 80, 80])
            upper_red2 = np.array([179, 255, 255])
            return cv2.bitwise_or(
                cv2.inRange(hsv, lower_red1, upper_red1),
                cv2.inRange(hsv, lower_red2, upper_red2),
            )

        lower_blue = np.array([100, 90, 40])
        upper_blue = np.array([130, 255, 220])
        return cv2.inRange(hsv, lower_blue, upper_blue)

    def estimate_cube_distance_mm(self, bbox_width, bbox_height):
        pixel_size = max(bbox_width, bbox_height)
        if pixel_size <= 0:
            return None
        raw_distance_mm = self.cube_size_mm * self.focal_length_px / pixel_size
        return raw_distance_mm * self.distance_correction_scale

    def draw_candidates(self, frame, candidates, selected):
        for candidate in candidates:
            x, y, w, h = candidate["bbox"]
            object_u = candidate["object_u"]
            object_v = candidate["object_v"]
            is_selected = candidate is selected
            color = (0, 255, 0) if is_selected else (180, 180, 180)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.circle(frame, (int(object_u), int(object_v)), 5, color, -1)
            cv2.putText(
                frame,
                str(candidate["number"]),
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                color,
                2,
            )

    @staticmethod
    def bbox_contains(outer, inner, margin=4):
        outer_x, outer_y, outer_w, outer_h = outer
        inner_x, inner_y, inner_w, inner_h = inner
        return (
            inner_x >= outer_x - margin
            and inner_y >= outer_y - margin
            and inner_x + inner_w <= outer_x + outer_w + margin
            and inner_y + inner_h <= outer_y + outer_h + margin
        )

    @staticmethod
    def bbox_area(bbox):
        return bbox[2] * bbox[3]

    @staticmethod
    def bbox_center(bbox):
        x, y, w, h = bbox
        return x + w / 2.0, y + h / 2.0

    @staticmethod
    def bbox_iou(a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b

        inter_x1 = max(ax, bx)
        inter_y1 = max(ay, by)
        inter_x2 = min(ax + aw, bx + bw)
        inter_y2 = min(ay + ah, by + bh)

        inter_w = max(0, inter_x2 - inter_x1)
        inter_h = max(0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h
        if inter_area <= 0:
            return 0.0

        union_area = aw * ah + bw * bh - inter_area
        if union_area <= 0:
            return 0.0

        return inter_area / union_area

    @staticmethod
    def point_inside_bbox(point_u, point_v, bbox, margin=0):
        x, y, w, h = bbox
        return (
            x - margin <= point_u <= x + w + margin
            and y - margin <= point_v <= y + h + margin
        )

    def suppress_inner_candidates(self, candidates):
        filtered = []
        sorted_candidates = sorted(
            candidates,
            key=lambda item: self.bbox_area(item["bbox"]),
            reverse=True,
        )

        for candidate in sorted_candidates:
            is_inner_box = False
            for kept in filtered:
                candidate_center_u, candidate_center_v = self.bbox_center(candidate["bbox"])
                candidate_area = self.bbox_area(candidate["bbox"])
                kept_area = self.bbox_area(kept["bbox"])

                if (
                    self.bbox_contains(kept["bbox"], candidate["bbox"], margin=8)
                    or (
                        candidate_area < kept_area * 0.75
                        and self.point_inside_bbox(
                            candidate_center_u,
                            candidate_center_v,
                            kept["bbox"],
                            margin=8,
                        )
                    )
                    or self.bbox_iou(kept["bbox"], candidate["bbox"]) > 0.45
                ):
                    is_inner_box = True
                    break
            if not is_inner_box:
                filtered.append(candidate)

        return filtered

    def assign_persistent_numbers(self, candidates):
        match_options = []

        for candidate_index, candidate in enumerate(candidates):
            candidate_diag = math.hypot(candidate["bbox_width"], candidate["bbox_height"])
            for number, track in self.object_tracks.items():
                distance = math.hypot(
                    candidate["object_u"] - track["object_u"],
                    candidate["object_v"] - track["object_v"],
                )
                iou = self.bbox_iou(candidate["bbox"], track["bbox"])
                track_diag = math.hypot(track["bbox"][2], track["bbox"][3])
                max_distance = max(
                    self.track_match_max_distance_px,
                    2.0 * max(candidate_diag, track_diag),
                )

                if iou <= 0.02 and distance > max_distance:
                    continue

                score = (1.0 - iou, distance)
                match_options.append((score, candidate_index, number))

        match_options.sort(key=lambda item: item[0])

        assigned_candidates = set()
        assigned_tracks = set()
        candidate_to_number = {}

        for _, candidate_index, number in match_options:
            if candidate_index in assigned_candidates or number in assigned_tracks:
                continue
            candidate_to_number[candidate_index] = number
            assigned_candidates.add(candidate_index)
            assigned_tracks.add(number)

        for candidate_index, candidate in enumerate(candidates):
            if candidate_index not in candidate_to_number:
                candidate_to_number[candidate_index] = self.next_track_number
                self.next_track_number += 1
            candidate["number"] = candidate_to_number[candidate_index]

        updated_tracks = {}
        for number, track in self.object_tracks.items():
            if number not in assigned_tracks:
                missed = track.get("missed", 0) + 1
                if missed <= self.track_max_missed_frames:
                    kept_track = dict(track)
                    kept_track["missed"] = missed
                    updated_tracks[number] = kept_track

        for candidate in candidates:
            updated_tracks[candidate["number"]] = {
                "object_u": candidate["object_u"],
                "object_v": candidate["object_v"],
                "bbox": candidate["bbox"],
                "missed": 0,
            }

        self.object_tracks = updated_tracks

    def age_unmatched_tracks(self):
        updated_tracks = {}
        for number, track in self.object_tracks.items():
            missed = track.get("missed", 0) + 1
            if missed <= self.track_max_missed_frames:
                kept_track = dict(track)
                kept_track["missed"] = missed
                updated_tracks[number] = kept_track
        self.object_tracks = updated_tracks

    def draw_state(self, output, aligned_u, aligned_v, approach_done, approach_status, now):
        if self.insert_sequence_done:
            status_text = "GRIP DONE"
        elif self.auto_approach and not self.approach_released:
            if aligned_u and aligned_v:
                held_seconds = 0.0
                if self.align_hold_start is not None:
                    held_seconds = min(self.auto_align_hold_sec, now - self.align_hold_start)
                status_text = f"CENTER HOLD {held_seconds:.1f}/{self.auto_align_hold_sec:.1f}s"
            else:
                status_text = "ALIGNING TARGET"
        elif self.auto_approach and self.approach_released:
            status_text = "READY TO GRIP" if approach_done else approach_status or "IK APPROACHING"
        elif aligned_u and aligned_v:
            status_text = "CENTER ALIGNED"
        elif aligned_u:
            status_text = "U aligned / adjusting V"
        elif aligned_v:
            status_text = "V aligned / adjusting U"
        else:
            status_text = "Adjusting U/V"

        cv2.putText(
            output,
            status_text,
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

    def update_s0_by_error(self, error_u, current_s0):
        if abs(error_u) <= self.center_tolerance_u:
            return current_s0, True

        if error_u > 0:
            current_s0 -= self.s0_step
        else:
            current_s0 += self.s0_step

        return clamp(current_s0, 0, 180), False

    def update_s12_by_error(self, error_v, current_s1, current_s2, current_s3):
        if abs(error_v) <= self.center_tolerance_v:
            return current_s1, current_s2, current_s3, True

        direction = 1.0 if error_v > 0 else -1.0
        direction *= self.v_direction
        current_s1 += direction * self.v_step * self.s1_weight
        current_s2 += direction * self.v_step * self.s2_weight

        return (
            clamp(current_s1, 0, 180),
            clamp(current_s2, 0, 180),
            clamp(current_s3, 0, 180),
            False,
        )

    def servo_to_link_angles(self, current_s1, current_s2, current_s3):
        a1 = 180.0 - current_s1
        a2 = a1 + self.link2_angle_at_90_deg - current_s2
        a3 = a2 + self.link3_offset_deg - current_s3
        return a1, a2, a3

    def link_angles_to_servo(self, a1, a2, a3):
        target_s1 = 180.0 - a1
        target_s2 = a1 + self.link2_angle_at_90_deg - a2
        target_s3 = a2 + self.link3_offset_deg - a3
        return (
            clamp(target_s1, 0, 180),
            clamp(target_s2, 0, 180),
            clamp(target_s3, 0, 180),
        )

    def forward_kinematics(self, current_s1, current_s2, current_s3):
        a1, a2, a3 = self.servo_to_link_angles(current_s1, current_s2, current_s3)
        a1_rad = math.radians(a1)
        a2_rad = math.radians(a2)
        a3_rad = math.radians(a3)
        r = (
            self.d1_mm * math.cos(a1_rad)
            + self.d2_mm * math.cos(a2_rad)
            + self.d3_mm * math.cos(a3_rad)
        )
        z = (
            self.base_to_link1_z_mm
            + self.d1_mm * math.sin(a1_rad)
            + self.d2_mm * math.sin(a2_rad)
            + self.d3_mm * math.sin(a3_rad)
        )
        return r, z, a1, a2, a3

    def solve_planar_ik(self, target_r, target_z, wrist_angle_deg):
        wrist_rad = math.radians(wrist_angle_deg)
        wrist_r = target_r - self.d3_mm * math.cos(wrist_rad)
        wrist_z = target_z - self.base_to_link1_z_mm - self.d3_mm * math.sin(wrist_rad)
        distance_sq = wrist_r * wrist_r + wrist_z * wrist_z
        cos_q2 = (
            distance_sq - self.d1_mm * self.d1_mm - self.d2_mm * self.d2_mm
        ) / (2.0 * self.d1_mm * self.d2_mm)

        if cos_q2 < -1.0 or cos_q2 > 1.0:
            return None

        q2 = -math.acos(cos_q2)
        q1 = math.atan2(wrist_z, wrist_r) - math.atan2(
            self.d2_mm * math.sin(q2),
            self.d1_mm + self.d2_mm * math.cos(q2),
        )
        a1 = math.degrees(q1)
        a2 = math.degrees(q1 + q2)
        a3 = wrist_angle_deg
        return self.link_angles_to_servo(a1, a2, a3)

    def update_approach_by_ik(self, error_v, current_s1, current_s2, current_s3):
        current_r, current_z, _, _, current_a3 = self.forward_kinematics(
            current_s1, current_s2, current_s3
        )

        if (
            abs(current_r - self.target_object_r_mm) <= self.ik_approach_r_step_mm
            and abs(current_z - self.grip_target_z_mm) <= self.ik_approach_z_step_mm
        ):
            return current_s1, current_s2, current_s3, True, "APPROACH DONE"

        next_r = current_r + clamp(
            self.target_object_r_mm - current_r,
            -self.ik_approach_r_step_mm,
            self.ik_approach_r_step_mm,
        )
        next_z = current_z + clamp(
            self.grip_target_z_mm - current_z,
            -self.ik_approach_z_step_mm,
            self.ik_approach_z_step_mm,
        )
        wrist_delta = clamp(
            error_v * self.ik_wrist_from_image_gain,
            -self.ik_wrist_step_limit_deg,
            self.ik_wrist_step_limit_deg,
        )
        target_angles = self.solve_planar_ik(next_r, next_z, current_a3 + wrist_delta)
        if target_angles is None:
            return current_s1, current_s2, current_s3, False, "IK UNREACHABLE"

        target_s1, target_s2, target_s3 = target_angles
        return target_s1, target_s2, target_s3, False, "IK APPROACHING"

    def run_insert_sequence(self):
        self.sequence_running = True
        try:
            self.gripper = self.gripper_open_angle
            self.publish_angles(self.current_pose())
            time.sleep(self.gripper_open_wait_sec)

            self.s3 = clamp(self.s3 + self.insert_final_s3_offset_deg, 0, 180)
            self.publish_angles(self.current_pose())
            time.sleep(self.insert_joint_step_wait_sec)

            self.s2 = clamp(self.s2 + self.insert_final_s2_offset_deg, 0, 180)
            self.publish_angles(self.current_pose())
            time.sleep(self.insert_joint_step_wait_sec)

            self.s1 = clamp(self.s1 + self.insert_final_s1_offset_deg, 0, 180)
            self.publish_angles(self.current_pose())
            time.sleep(self.gripper_close_before_wait_sec)

            self.gripper = self.gripper_close_angle
            self.publish_angles(self.current_pose())
            time.sleep(self.return_to_start_delay_sec)

            self.s1 = self.drop_pose[1]
            self.publish_angles(self.current_pose())
            time.sleep(self.insert_joint_step_wait_sec)

            self.s0, self.s1, self.s2, self.s3, self.gripper = self.drop_pose
            self.publish_angles(self.current_pose())
            time.sleep(self.insert_joint_step_wait_sec)

            self.gripper = self.gripper_open_angle
            self.publish_angles(self.current_pose())
            time.sleep(self.gripper_open_wait_sec)

            self.gripper = self.gripper_close_angle
            self.publish_angles(self.current_pose())
            time.sleep(self.insert_joint_step_wait_sec)

            self.s0, self.s1, self.s2, self.s3, self.gripper = self.start_pose
            self.publish_angles(self.current_pose())
        finally:
            self.sequence_running = False

    def should_auto_start(self, now):
        return (
            self.auto_start_enabled
            and not self.auto_approach
            and not self.sequence_running
            and not self.approach_stop_latched
            and now - self.search_pose_enter_time >= self.auto_start_delay_sec
        )

    def start_auto(self, result=None, reason="auto approach on"):
        if result is not None:
            self.target_lock = (result["object_u"], result["object_v"])
        self.auto_approach = True
        self.filtered_error_v = 0.0
        self.insert_sequence_done = False
        self.approach_stop_latched = False
        self.align_hold_start = None
        self.approach_released = False
        self.publish_status(reason)

    def stop_auto(self):
        self.auto_approach = False
        self.target_lock = None
        self.align_hold_start = None
        self.approach_released = False
        self.approach_stop_latched = False
        self.search_pose_enter_time = time.monotonic()
        self.publish_status("auto approach off")

    def reset_to_start(self):
        self.s0, self.s1, self.s2, self.s3, self.gripper = self.start_pose
        self.insert_sequence_done = False
        self.approach_stop_latched = False
        self.auto_approach = False
        self.target_lock = None
        self.object_tracks = {}
        self.next_track_number = 1
        self.align_hold_start = None
        self.approach_released = False
        self.search_pose_enter_time = time.monotonic()
        self.publish_angles(self.current_pose())
        self.publish_status(
            f"reset to start pose; waiting {self.auto_start_delay_sec:.1f}s"
        )

    def set_target_color(self, color):
        if color not in ("red", "blue"):
            return
        self.target_color = color
        self.auto_approach = False
        self.target_lock = None
        self.approach_stop_latched = False
        self.object_tracks = {}
        self.next_track_number = 1
        self.align_hold_start = None
        self.approach_released = False
        self.search_pose_enter_time = time.monotonic()
        self.publish_status(f"target color: {color}")

    def publish_angles(self, angles):
        msg = Int32MultiArray()
        msg.data = [int(round(clamp(value, 0, 180))) for value in angles]
        self.servo_pub.publish(msg)
        self.last_servo_publish_time = time.monotonic()
        self.get_logger().info(f"servo target: {msg.data}")

    def republish_current_pose_if_needed(self, now):
        if self.servo_republish_interval_sec <= 0:
            return

        if now - self.last_servo_publish_time >= self.servo_republish_interval_sec:
            self.publish_angles(self.current_pose())

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    @staticmethod
    def pose_changed(old_pose, new_pose):
        return any(abs(old - new) >= 0.1 for old, new in zip(old_pose, new_pose))

    def destroy_node(self):
        if self.camera is not None:
            self.camera.close()
        if self.display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmVisionGripper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
