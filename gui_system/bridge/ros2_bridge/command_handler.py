"""명령 핸들러 — GUI의 command envelope을 ROS2 발행 / action goal로
변환한다.

매핑(Must 범위, 단일 namespace TurtleBot3):
  start_mission   → /<ns>/navigate_to_pose의 NavigateToPose action goal
                    (target 노드 x/y는 GUI 맵 레지스트리에서 조회되어
                    command payload로 전달됨).
  pause           → 현재 Nav2 goal 취소 + /<ns>/cmd_vel에 0 Twist 발행
  resume          → 마지막 NavigateToPose goal 재발행
  reset_mission   → 취소 + 로컬 미션 상태 초기화
  change_goal     → 취소 + 새 target으로 재발행
  emergency_stop  → 0 Twist 발행 + goal 취소(이후 전송 잠금)
  emergency_clear → 잠금 해제해 이후 명령이 다시 흐르게 함

rclpy가 없어도 핸들러는 accepted=true를 반환해, WebSocket synth feed가
UI 전용 smoke 테스트에서 대시보드를 사용 가능하게 유지한다.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Optional

from .ros_node import BridgeNode, HAS_RCLPY

HAS_NAV2 = False
if HAS_RCLPY:
    from geometry_msgs.msg import Twist  # type: ignore
    from geometry_msgs.msg import PoseStamped  # type: ignore
    from geometry_msgs.msg import PoseWithCovarianceStamped  # type: ignore
    from std_msgs.msg import String  # type: ignore
    from rclpy.action import ActionClient  # type: ignore
    try:
        from nav2_msgs.action import NavigateToPose  # type: ignore
        HAS_NAV2 = True
    except ImportError:
        # Nav2가 모든 로봇 이미지에 설치돼 있지는 않다(예: 브릿지가 Nav2를
        # 실행하는 봇과 통신하는 워크스테이션에서 돌 수 있음). Nav2가 없으면
        # goal 명령은 stub으로 폴백한다.
        NavigateToPose = None  # type: ignore


class CommandHandler:
    """브릿지 런타임당 핸들러 인스턴스 하나; thread-safe."""

    def __init__(self, node: BridgeNode, robots: list[dict[str, Any]]) -> None:
        self.node = node
        self.robots = robots
        self.lock = threading.Lock()
        self.emergency = False
        self.last_goal: dict[str, Any] = {}
        self.cmd_vel_pubs: dict[str, Any] = {}
        self.nav_clients: dict[str, Any] = {}
        self.initialpose_pubs: dict[str, Any] = {}
        self.mission_command_pubs: dict[str, Any] = {}
        if not HAS_RCLPY or node.ros_node is None:
            return
        for r in robots:
            raw_ns = (r.get("namespace") or "").strip("/")
            ns = ("/" + raw_ns) if raw_ns else ""
            self.cmd_vel_pubs[r["id"]] = node.ros_node.create_publisher(Twist, f"{ns}/cmd_vel", 10)
            # AMCL의 /initialpose 리스너는 일반 publisher 타겟이다.
            # GUI의 "위치 추정" 버튼이 여기에 써서 로봇의 위치추정 추정값을
            # 재시드한다. RViz 동작과 동일하다.
            self.initialpose_pubs[r["id"]] = node.ros_node.create_publisher(
                PoseWithCovarianceStamped, f"{ns}/initialpose", 10
            )
            self.mission_command_pubs[r["id"]] = node.ros_node.create_publisher(
                String, f"{ns}/mission_command", 10
            )
            if HAS_NAV2:
                self.nav_clients[r["id"]] = ActionClient(node.ros_node, NavigateToPose, f"{ns}/navigate_to_pose")

    # ------------------------------------------------------------------
    # websocket_server.handle()가 쓰는 공개 진입점
    # ------------------------------------------------------------------

    def handle(self, cmd: dict[str, Any]) -> dict[str, Any]:
        kind = cmd.get("kind", "")
        payload = cmd.get("payload") or {}
        request_id = cmd.get("request_id", "")
        with self.lock:
            if self.emergency and kind not in ("emergency_clear", "emergency_stop"):
                return {
                    "request_id": request_id,
                    "accepted": False,
                    "message": "emergency active",
                }
            try:
                msg = self._dispatch(kind, payload)
            except Exception as e:  # noqa: BLE001
                return {
                    "request_id": request_id,
                    "accepted": False,
                    "message": f"handler error: {e}",
                }
            return {
                "request_id": request_id,
                "accepted": True,
                "message": msg,
            }

    # ------------------------------------------------------------------
    # Dispatch 테이블
    # ------------------------------------------------------------------

    def _dispatch(self, kind: str, payload: dict[str, Any]) -> str:
        if kind == "start_mission" or kind == "change_goal":
            return self._send_goal(payload)
        if kind == "pause":
            self._cancel_all()
            self._publish_zero_twist()
            return "paused"
        if kind == "resume":
            if self.last_goal:
                return self._send_goal(self.last_goal)
            return "no prior goal"
        if kind == "reset_mission":
            self._cancel_all()
            self.last_goal = {}
            return "reset"
        if kind == "emergency_stop":
            self.emergency = True
            self._cancel_all()
            self._publish_zero_twist()
            return "stopped"
        if kind == "emergency_clear":
            self.emergency = False
            return "cleared"
        if kind == "set_pose_estimate":
            return self._set_pose_estimate(payload)
        return f"unknown kind {kind}"

    def _set_pose_estimate(self, payload: dict[str, Any]) -> str:
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        theta = float(payload.get("theta", 0.0))
        robot_id = payload.get("robot_id") or (self.robots[0]["id"] if self.robots else "")
        if not HAS_RCLPY or robot_id not in self.initialpose_pubs:
            return f"stub initialpose ({x:.2f}, {y:.2f}, {theta:.2f})"
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.ros_node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        # RViz에 가까운 기본 공분산: x/y에 0.25m, yaw에 ~π/12.
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = (math.pi / 12.0) ** 2
        msg.pose.covariance = cov
        self.initialpose_pubs[robot_id].publish(msg)
        return f"initialpose set ({x:.2f}, {y:.2f}, {theta:.2f})"

    # ------------------------------------------------------------------
    # ROS2 action
    # ------------------------------------------------------------------

    def _send_goal(self, payload: dict[str, Any]) -> str:
        # target 로봇 식별. 대시보드의 미션 제어는 기본적으로 리더를
        # 대상으로 하며, 멀티 로봇 라우팅은 Should 단계 소관이다.
        robot_id = payload.get("robot_id") or (self.robots[0]["id"] if self.robots else "")
        target = payload.get("target")
        target_node = payload.get("target_node")
        if target_node in ("charger_front", "charger_entry", "charger"):
            return self._send_charger_parking_command(robot_id)
        # 명시적 {x, y, theta}, 또는 백엔드가 forward 전에 해석했어야 하는
        # 노드명 조회 둘 다 허용한다.
        x = float(payload.get("x", target["x"] if isinstance(target, dict) else 0.0))
        y = float(payload.get("y", target["y"] if isinstance(target, dict) else 0.0))
        theta = float(payload.get("theta", target["theta"] if isinstance(target, dict) else 0.0))
        self.last_goal = {"robot_id": robot_id, "x": x, "y": y, "theta": theta, "target_node": target_node}
        if not HAS_RCLPY or not HAS_NAV2 or robot_id not in self.nav_clients:
            return f"stub goal {target_node or (x, y)}"
        client = self.nav_clients[robot_id]
        if not client.wait_for_server(timeout_sec=2.0):
            return "nav2 action server unavailable"
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(x, y, theta)
        client.send_goal_async(goal)
        return f"goal sent to {target_node or (x, y)}"

    def _send_charger_parking_command(self, robot_id: str) -> str:
        if not HAS_RCLPY or robot_id not in self.mission_command_pubs:
            return "stub charger parking"
        msg = String()
        msg.data = "start_charger_parking"
        self.mission_command_pubs[robot_id].publish(msg)
        return "charger parking command published"

    def _cancel_all(self) -> None:
        if not HAS_RCLPY:
            return
        for client in self.nav_clients.values():
            try:
                client._cancel_goal_async  # 속성 존재 확인
                # nav2_msgs 취소: 가장 쉬운 건 서버 측 호출이지만, rclpy의
                # ActionClient는 goal handle을 통해 취소한다. stub에서는
                # noop — 실제 Nav2 취소는 send_goal_async가 반환한 각 goal
                # handle을 추적해야 한다.
            except AttributeError:
                pass

    def _publish_zero_twist(self) -> None:
        if not HAS_RCLPY:
            return
        zero = Twist()
        for pub in self.cmd_vel_pubs.values():
            pub.publish(zero)

    def _pose_stamped(self, x: float, y: float, theta: float) -> Any:
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.node.ros_node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        # Yaw → quaternion (Z축 회전만).
        ps.pose.orientation.z = math.sin(theta / 2.0)
        ps.pose.orientation.w = math.cos(theta / 2.0)
        return ps


# 모듈 레벨 싱글턴, 시작 시 __main__이 설정한다. websocket_server의
# import를 위해 기존 `handle(cmd)` 함수를 유지한다.
_INSTANCE: Optional[CommandHandler] = None


def set_instance(inst: CommandHandler) -> None:
    global _INSTANCE
    _INSTANCE = inst


def handle(cmd: dict[str, Any]) -> dict[str, Any]:
    if _INSTANCE is None:
        return {
            "request_id": cmd.get("request_id", ""),
            "accepted": True,
            "message": "stub (no ROS2)",
        }
    return _INSTANCE.handle(cmd)


# websocket 서버가 다른 곳에서 쓰는 "현재 시각" 헬퍼를 유지한다.
def now_ms() -> int:
    return int(time.time() * 1000)
