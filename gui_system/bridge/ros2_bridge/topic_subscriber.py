"""GUI 브릿지용 ROS2 토픽 구독자.

대시보드가 관심 있는 토픽을 로봇별로 구독하고, 각 ROS 메시지를 Go
gateway 프로토콜과 동일한 envelope으로 포장해 sink 콜백(WebSocket
broadcast hook)에 넘긴다.

토픽 선택은 TurtleBot3 네비게이션 스택을 반영한다:
  - /<ns>/amcl_pose  (geometry_msgs/PoseWithCovarianceStamped) — AMCL이
    로봇을 위치추정한 후의 pose. pose가 발행되지 않으면 /<ns>/odom로 폴백.
  - /<ns>/odom       (nav_msgs/Odometry)                       — 속도 + pose
  - /<ns>/scan       (sensor_msgs/LaserScan)                   — scan 기반
    장애물 레이어용; 틱마다 수백 개 range를 밀지 않도록 가벼운 요약으로 전송.
  - /<ns>/plan       (nav_msgs/Path)                           — Nav2 global plan
  - /<ns>/battery_state (sensor_msgs/BatteryState, optional)   — 배터리 %

rclpy가 없을 때도 import-safe하다 — 단순히 noop이 된다.
"""
from __future__ import annotations

import math
import time
from typing import Any, Callable, Iterable

from .ros_node import BridgeNode, HAS_RCLPY

HAS_TF = False
if HAS_RCLPY:
    import rclpy  # type: ignore
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist  # type: ignore  # noqa: F401
    from nav_msgs.msg import Odometry, Path  # type: ignore
    from sensor_msgs.msg import BatteryState, LaserScan  # type: ignore
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy  # type: ignore
    try:
        import tf2_ros  # type: ignore
        HAS_TF = True
    except Exception:  # pragma: no cover - tf2_ros 미설치 환경 대비
        HAS_TF = False


Sink = Callable[[dict[str, Any]], None]


def _envelope(typ: str, robot_id: str, payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": typ,
        "robot_id": robot_id,
        "timestamp": int(time.time() * 1000),
        "seq": 0,
        "version": 1,
        "payload": payload,
    }


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    # ROS quaternion → 평면 yaw(Z축 회전).
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class TopicSubscriber:
    """단일 sink로 공급하는 로봇별 ROS2 구독."""

    def __init__(self, node: BridgeNode, robots: Iterable[dict[str, str]], sink: Sink) -> None:
        self.node = node
        self.sink = sink
        self.subs: list[Any] = []
        self._last_pose: dict[str, tuple[float, float, float]] = {}
        self._last_battery: dict[str, float] = {}
        self._last_telemetry_ts: dict[str, float] = {}
        # 주행 동작 로그용 상태: 로봇별 마지막 동작 라벨과 마지막 emit 시각.
        self._last_motion: dict[str, str] = {}
        self._last_motion_ts: dict[str, float] = {}
        # TF 기반 pose 상태: 로봇별 마지막 TF push 시각(폴백 우선순위 판정용),
        # TF buffer/listener, (robot_id, base_frame) 매핑, map 프레임.
        self._tf_active: dict[str, float] = {}
        self._tf_buffer: Any = None
        self._tf_listener: Any = None
        self._tf_robots: list[tuple[str, str]] = []
        self._map_frame = "map"
        if not HAS_RCLPY or node.ros_node is None:
            return
        robots = list(robots)
        for r in robots:
            self._wire(r["id"], r.get("namespace", r["id"]))
        self._setup_tf(node, robots)

    def _wire(self, robot_id: str, namespace: str) -> None:
        raw_ns = (namespace or "").strip("/")
        ns = ("/" + raw_ns) if raw_ns else ""

        # Pose: AMCL을 우선하되 AMCL이 안 돌면 odom을 받는다.
        pose_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.subs.append(
            self.node.ros_node.create_subscription(
                PoseWithCovarianceStamped,
                f"{ns}/amcl_pose",
                lambda msg, rid=robot_id: self._on_amcl(rid, msg),
                pose_qos,
            )
        )

        sensor_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.subs.append(
            self.node.ros_node.create_subscription(
                Odometry,
                f"{ns}/odom",
                lambda msg, rid=robot_id: self._on_odom(rid, msg),
                sensor_qos,
            )
        )

        self.subs.append(
            self.node.ros_node.create_subscription(
                LaserScan,
                f"{ns}/scan",
                lambda msg, rid=robot_id: self._on_scan(rid, msg),
                sensor_qos,
            )
        )

        # 주행로직(C++ DWA)이 동적 장애물 우회 호를 nav_msgs/Path로 발행한다는
        # 가정의 구독. 토픽명은 amr_interfaces 계약 합의 시 변경될 수 있음.
        # publisher가 아직 없으면 콜백은 호출되지 않으므로 무영향.
        self.subs.append(
            self.node.ros_node.create_subscription(
                Path,
                f"{ns}/avoidance_path",
                lambda msg, rid=robot_id: self._on_avoidance_path(rid, msg),
                pose_qos,
            )
        )

        # 주행로직(C++)이 자체 산출한 전역 계획 경로(nav_msgs/Path) 구독.
        # 활성화 전엔 백엔드 mission.go의 로컬 Dijkstra가 path_data로 점선을
        # 표시하고, publisher가 시작되면 같은 frontend 키에 latest-wins로
        # 덮어써져 표시 출처가 주행 측으로 자동 전환된다.
        self.subs.append(
            self.node.ros_node.create_subscription(
                Path,
                f"{ns}/planned_path",
                lambda msg, rid=robot_id: self._on_planned_path(rid, msg),
                pose_qos,
            )
        )

        try:
            self.subs.append(
                self.node.ros_node.create_subscription(
                    BatteryState,
                    f"{ns}/battery_state",
                    lambda msg, rid=robot_id: self._on_battery(rid, msg),
                    sensor_qos,
                )
            )
        except Exception:
            # battery_state는 tb3 버거에서 선택 사항이다.
            pass

    def _setup_tf(self, node: BridgeNode, robots: list[dict[str, str]]) -> None:
        # TF map->base_footprint를 일정한 고빈도(20Hz)로 떠 robot_pose를 구동한다.
        # amcl_pose는 발행이 불규칙·저빈도이고 보정 시 위치가 점프해 맵 위 마커가
        # 끊겨 보였다. TF는 odom 빈도로 매끄럽고 map 프레임으로 정합되며(주행로직
        # mission_loop이 쓰는 바로 그 소스), 그래서 마커가 부드럽게 움직인다.
        # tf2_ros가 없거나 lookup이 실패하면 기존 amcl/odom 폴백이 처리한다.
        if not HAS_TF:
            return
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, node.ros_node)
        for r in robots:
            raw_ns = (r.get("namespace") or "").strip("/")
            base = r.get("base_frame") or (f"{raw_ns}/base_footprint" if raw_ns else "base_footprint")
            self._tf_robots.append((r["id"], base))
        node.ros_node.create_timer(0.05, self._on_tf_timer)  # 20Hz

    def _on_tf_timer(self) -> None:
        if self._tf_buffer is None:
            return
        now = time.time()
        for robot_id, base_frame in self._tf_robots:
            try:
                tf = self._tf_buffer.lookup_transform(self._map_frame, base_frame, rclpy.time.Time())
            except Exception:
                # TF 아직 준비 안 됨(AMCL 미가동 등) → amcl/odom 폴백이 채운다.
                continue
            # TF 신선도 검사 — GUI PC에 AMCL만 떠 있고 봇 본체가 죽으면
            # AMCL이 발행하는 map→odom은 계속 살아 있는데 odom→base_footprint
            # 가지가 갱신 안 돼서 chain lookup이 stale stamp 그대로 성공한다.
            # 그래서 stale x/y가 매 50ms 영구히 sink로 흘러 robot이 online으로
            # 보이는 가짜 heartbeat 버그가 났다. transform stamp가 _TF_STALE_SEC
            # 이상 묵은 거면 emit을 끊어 프런트 STALE_MS 가드가 offline 전이를
            # 잡게 한다.
            stamp = tf.header.stamp
            stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
            age = now - stamp_sec
            if stamp_sec > 0.0 and age > self._TF_STALE_SEC:
                continue
            tr = tf.transform.translation
            q = tf.transform.rotation
            x = float(tr.x)
            y = float(tr.y)
            theta = _yaw_from_quat(q.x, q.y, q.z, q.w)
            self._last_pose[robot_id] = (x, y, theta)
            self._tf_active[robot_id] = now
            self.sink(self._pose_envelope(robot_id))

    # 봇이 살아있을 때 odom→base_footprint는 ~30Hz, AMCL map→odom은 ~10Hz로
    # 발행되니 1.5초면 정상 동작 중 오탐 없이 봇 다운을 잡는다.
    _TF_STALE_SEC = 1.5

    def _tf_is_active(self, robot_id: str) -> bool:
        # 최근 0.5s 내 TF push가 있었으면 TF가 마커를 구동 중인 것으로 본다.
        return (time.time() - self._tf_active.get(robot_id, 0.0)) < 0.5

    def _on_amcl(self, robot_id: str, msg: Any) -> None:
        # AMCL 메시지 stamp가 _TF_STALE_SEC 이상 묵었으면 폐기 — TF 폴백과
        # 동일한 이유로, 봇이 죽은 후에도 AMCL이 재발행하는 stale pose를
        # 영구 heartbeat로 흘려보내지 않도록.
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
        age = time.time() - stamp_sec
        if stamp_sec > 0.0 and age > self._TF_STALE_SEC:
            return
        p = msg.pose.pose
        x = float(p.position.x)
        y = float(p.position.y)
        theta = _yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        self._last_pose[robot_id] = (x, y, theta)
        # TF가 마커를 구동 중이면(고빈도·매끄러움) amcl push는 생략한다. 둘 다
        # push하면 불규칙한 amcl 갱신이 끼어들어 다시 점프·끊김이 보인다.
        if self._tf_is_active(robot_id):
            return
        self.sink(self._pose_envelope(robot_id))

    def _on_odom(self, robot_id: str, msg: Any) -> None:
        # AMCL/TF가 조용하면 odom이 pose 채널을 구동한다. RC카처럼
        # map 프레임 odom만 있는 로봇은 이 경로로 GUI 마커를 계속 갱신한다.
        if not self._tf_is_active(robot_id):
            p = msg.pose.pose
            x = float(p.position.x)
            y = float(p.position.y)
            theta = _yaw_from_quat(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            self._last_pose[robot_id] = (x, y, theta)
            self.sink(self._pose_envelope(robot_id))

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)

        # 주행 동작(전진/후진/좌·우회전/정지)을 분류해, 상태가 바뀔 때만
        # status 이벤트로 emit한다 — 대시보드 로그창에 한 줄씩 남기기 위함.
        # odom 풀 레이트로 분류하되 emit은 변화 시점에만 하므로 폭주하지 않는다.
        self._emit_motion_if_changed(robot_id, vx, wz)

        # 속도 → telemetry (~2Hz로 스로틀).
        now = time.time()
        if now - self._last_telemetry_ts.get(robot_id, 0) < 0.5:
            return
        self._last_telemetry_ts[robot_id] = now
        speed = math.hypot(vx, vy)
        self.sink({
            "type": "telemetry",
            "robot_id": robot_id,
            "timestamp": int(now * 1000),
            "seq": 0,
            "version": 1,
            "payload": {
                "robot_id": robot_id,
                "timestamp": int(now * 1000),
                "velocity": speed,
                "battery": self._last_battery.get(robot_id, 0.0),
            },
        })

    # 동작 분류 임계값. 이보다 작은 속도는 잡음/정지로 본다.
    _LIN_DEADBAND = 0.04  # m/s
    _ANG_DEADBAND = 0.20  # rad/s

    @staticmethod
    def _classify_motion(vx: float, wz: float) -> str:
        moving = abs(vx) >= TopicSubscriber._LIN_DEADBAND
        turning = abs(wz) >= TopicSubscriber._ANG_DEADBAND
        if not moving and not turning:
            return "stopped"
        # 후진은 회전 동반 여부와 무관하게 우선 표기한다.
        if moving and vx < 0:
            return "reverse"
        # ROS 관례상 angular.z > 0 은 반시계(좌회전), < 0 은 시계(우회전).
        if turning:
            return "turn_left" if wz > 0 else "turn_right"
        return "forward"

    def _emit_motion_if_changed(self, robot_id: str, vx: float, wz: float) -> None:
        state = self._classify_motion(vx, wz)
        prev = self._last_motion.get(robot_id)
        if state == prev:
            return
        # 최초 관측이 '정지'면 기준선만 기록하고 로그는 남기지 않는다(시작 시
        # 세 로봇이 일제히 '정지'를 찍는 잡음 방지).
        if prev is None and state == "stopped":
            self._last_motion[robot_id] = state
            return
        # 임계 부근 떨림으로 인한 로그 폭주를 막기 위해 로봇별 최소 0.3s 간격.
        now = time.time()
        if now - self._last_motion_ts.get(robot_id, 0.0) < 0.3:
            return
        self._last_motion[robot_id] = state
        self._last_motion_ts[robot_id] = now
        self.sink({
            "type": "status",
            "robot_id": robot_id,
            "timestamp": int(now * 1000),
            "seq": 0,
            "version": 1,
            "payload": {"event_type": f"motion_{state}", "robot_id": robot_id},
        })

    def _on_scan(self, robot_id: str, msg: Any) -> None:
        # 두 가지를 한 콜백에서 처리: (1) 근접 장애물 alert, (2) 시각화용
        # scan_points(map 프레임 점 클라우드).
        ranges = msg.ranges
        n = len(ranges)
        range_min = msg.range_min
        range_max = msg.range_max
        # alert: 유효 range 중 최소가 0.2m 미만이면 한 번 emit.
        min_r = float("inf")
        for r in ranges:
            if math.isfinite(r) and r > 0 and r < min_r:
                min_r = r
        if min_r < 0.20:
            self.sink({
                "type": "alert",
                "robot_id": robot_id,
                "timestamp": int(time.time() * 1000),
                "seq": 0,
                "version": 1,
                "payload": {"event_type": "obstacle_close", "robot_id": robot_id, "min_range": min_r},
            })
        # scan_points: laser frame의 (r,θ)를 TF로 map 프레임 (x,y)로 변환해 broadcast.
        # 프런트가 맵 위에 작은 초록 점으로 그린다. step=1로 전체 빔을 보낸다
        # (LDS-01 360빔 × 5Hz = 1800pts/s, WS 트래픽 부담 없음).
        if self._tf_buffer is None:
            return
        src_frame = msg.header.frame_id or "base_scan"
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, src_frame, rclpy.time.Time())
        except Exception:
            # TF가 아직 준비 안 됨(AMCL 미가동 등). alert는 이미 전송됐고 시각화는
            # 다음 사이클에 다시 시도한다.
            return
        tr = tf.transform.translation
        q = tf.transform.rotation
        cx = float(tr.x)
        cy = float(tr.y)
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        pts: list[dict[str, float]] = []
        step = 1
        for i in range(0, n, step):
            r = ranges[i]
            if not math.isfinite(r) or r < range_min or r > range_max:
                continue
            a = angle_min + i * angle_inc
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            # laser→map: yaw 회전 후 translation (2D).
            mx = cos_y * lx - sin_y * ly + cx
            my = sin_y * lx + cos_y * ly + cy
            pts.append({"x": mx, "y": my})
        if not pts:
            return
        self.sink({
            "type": "scan_points",
            "robot_id": robot_id,
            "timestamp": int(time.time() * 1000),
            "seq": 0,
            "version": 1,
            "payload": {"robot_id": robot_id, "points": pts},
        })

    def _on_battery(self, robot_id: str, msg: Any) -> None:
        pct = float(getattr(msg, "percentage", 0.0))
        if pct > 1.5:  # 일부 드라이버는 0..100으로 보고함
            pct /= 100.0
        pct = max(0.0, min(1.0, pct))
        # BatteryState.percentage는 ADC 노이즈로 1~2% 떨려 프런트 toFixed(0)
        # 표시가 끊임없이 바뀐다. EMA(α=0.08, 1Hz 발행 기준 시상수 ~12s)로
        # 평활해 정수 표시가 안정되게 한다. 첫 샘플은 그대로 채택.
        prev = self._last_battery.get(robot_id)
        if prev is None:
            self._last_battery[robot_id] = pct
        else:
            self._last_battery[robot_id] = prev * 0.92 + pct * 0.08

    def _on_avoidance_path(self, robot_id: str, msg: Any) -> None:
        # DWA가 선택한 우회 호(map 프레임 PoseStamped 배열)를 단순 점 리스트로
        # 줄여 local_path envelope으로 broadcast. 빈 poses면 빈 points로 보내
        # 프런트의 직전 호 선이 자연 소거된다(drawPaths가 length<2 스킵).
        pts = [
            {"x": float(ps.pose.position.x), "y": float(ps.pose.position.y)}
            for ps in msg.poses
        ]
        self.sink({
            "type": "local_path",
            "robot_id": robot_id,
            "timestamp": int(time.time() * 1000),
            "seq": 0,
            "version": 1,
            "payload": {"robot_id": robot_id, "points": pts},
        })

    def _on_planned_path(self, robot_id: str, msg: Any) -> None:
        # 주행로직이 산출한 전역 계획 경로(map 프레임 PoseStamped 배열)를
        # planned_path envelope으로 broadcast. 프런트에서 같은 global 키에
        # 적재돼 backend mission.go의 path_data를 자동으로 대체.
        pts = [
            {"x": float(ps.pose.position.x), "y": float(ps.pose.position.y)}
            for ps in msg.poses
        ]
        self.sink({
            "type": "planned_path",
            "robot_id": robot_id,
            "timestamp": int(time.time() * 1000),
            "seq": 0,
            "version": 1,
            "payload": {"robot_id": robot_id, "points": pts},
        })

    def _pose_envelope(self, robot_id: str) -> dict[str, Any]:
        x, y, theta = self._last_pose[robot_id]
        return _envelope("robot_pose", robot_id, {
            "robot_id": robot_id,
            "pose": {"x": x, "y": y, "theta": theta},
            "battery": self._last_battery.get(robot_id, 0.0),
            "status": "moving",
            "connection_state": "online",
        })
