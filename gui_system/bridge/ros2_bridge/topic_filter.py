"""ROS2 → 백엔드 forward 경로의 토픽 필터링·다운샘플링.

설계 §3-6에 따른 레이트:
    Pose 5Hz, TF 5Hz, Odom 5Hz, Costmap ≤1Hz, Path/MissionState 이벤트 기반.
이 모듈은 그 rate gate들을 조합 가능한 predicate로 구현한다.
"""
from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass
class RateGate:
    period_seconds: float
    _last_emit: float = 0.0

    def allow(self) -> bool:
        now = time.monotonic()
        if now - self._last_emit >= self.period_seconds:
            self._last_emit = now
            return True
        return False


def pose_gate() -> RateGate:
    return RateGate(period_seconds=0.2)


def costmap_gate() -> RateGate:
    return RateGate(period_seconds=1.0)
