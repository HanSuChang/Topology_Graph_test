"""rclpy 노드 래퍼.

asyncio 루프가 ROS2를 띄울지 말지 결정할 수 있도록 main.py와 분리해
둔다(Must 데모는 mock으로 동작하며 rclpy를 절대 열지 않는다). rclpy가
없으면 HAS_RCLPY는 False이고 — 브릿지의 나머지 코드는 import 실패 대신
이 플래그로 분기한다.
"""
from __future__ import annotations

from typing import Any

try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


class BridgeNode:
    """rclpy가 import 가능할 때만 rclpy Node를 지연 생성하는 얇은 래퍼.
    코드베이스의 나머지는 main.py의 BridgeRuntime 클래스를 통해 이것을
    호출한다.
    """

    def __init__(self, name: str = "gui_bridge") -> None:
        self.name = name
        self.ros_node: Any = None
        if HAS_RCLPY:
            rclpy.init()
            self.ros_node = Node(name)  # type: ignore

    def spin_once(self) -> None:
        if HAS_RCLPY and self.ros_node is not None:
            rclpy.spin_once(self.ros_node, timeout_sec=0)

    def shutdown(self) -> None:
        if HAS_RCLPY:
            if self.ros_node is not None:
                self.ros_node.destroy_node()
            rclpy.shutdown()
