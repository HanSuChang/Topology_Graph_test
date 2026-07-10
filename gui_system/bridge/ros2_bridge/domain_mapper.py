"""ROS2 메시지 → 도메인 dict 변환.

backend/internal/bridge/json_adapter.go를 반영한다: wire 스키마가
동일해 브릿지를 gRPC와 WebSocket 사이에서 교체해도 백엔드가 어느 쪽이
쓰이는지 알 필요가 없다.
"""
from __future__ import annotations

from typing import Any


def pose_to_dict(msg: Any) -> dict[str, Any]:
    """geometry_msgs/PoseStamped → 백엔드 RobotPose payload로 변환."""
    return {
        "pose": {
            "x": float(getattr(msg.pose.position, "x", 0)),
            "y": float(getattr(msg.pose.position, "y", 0)),
            "theta": 0.0,
        }
    }


def costmap_to_dict(msg: Any) -> dict[str, Any]:
    """nav_msgs/OccupancyGrid → costmap payload로 변환."""
    return {
        "origin": {"x": float(msg.info.origin.position.x), "y": float(msg.info.origin.position.y)},
        "resolution": float(msg.info.resolution),
        "width": int(msg.info.width),
        "height": int(msg.info.height),
        "data": list(msg.data),
    }
