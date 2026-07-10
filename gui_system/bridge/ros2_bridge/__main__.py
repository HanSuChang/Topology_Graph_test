"""진입점: `python -m ros2_bridge --config ../backend/configs/config.yaml`.

설계 §3-4에 따라 rclpy를 asyncio와 통합한다: 단일 이벤트 루프가 ROS2의
`spin_once`와 WS/gRPC 서버를 함께 구동한다. rclpy와 설정된 토픽에 닿으면
브릿지는 실제 /amcl_pose, /odom, /scan, /plan 메시지를 WebSocket fan-out
으로 forward하고, 그렇지 않으면 synth producer가 동작해 GUI가 렌더할
무언가를 유지한다.
"""
from __future__ import annotations

import argparse
import asyncio
import os

from . import command_handler, config
from .ros_node import BridgeNode, HAS_RCLPY
from .websocket_server import WebSocketServer
from .grpc_server import serve as grpc_serve
from .topic_subscriber import TopicSubscriber


def _default_robots(cfg: dict) -> list[dict]:
    """설정된 로봇 리스트를 반환하며, 미지정 시 글로벌 namespace의 단일
    기본 TurtleBot3로 폴백한다."""
    bridge_cfg = cfg.get("bridge_local") or cfg.get("bridge", {}) or {}
    ros_cfg = bridge_cfg.get("ros") or cfg.get("ros") or {}
    robots = ros_cfg.get("robots") or []
    if not robots:
        return [{"id": "tb3_leader", "namespace": ""}]
    return robots


async def amain(args: argparse.Namespace) -> None:
    # 2단계 config 로드: Go 백엔드 yaml이 트랜스포트(bridge.type +
    # bridge.address)의 진실 원천이고, 이 모듈 옆의 bridge-local yaml은
    # ROS 전용 설정(카메라, namespace)을 담는다.
    cfg = config.load(args.config)
    local_cfg = config.load(os.path.join(os.path.dirname(__file__), "..", "config.yaml"))
    if local_cfg:
        cfg.setdefault("bridge_local", local_cfg)
    host, port = config.bridge_address(cfg)
    kind = cfg.get("bridge", {}).get("type", "websocket")

    # launch 스크립트를 실행하는 운영자가 수동 export를 잊을 수 있으므로
    # ROS_DOMAIN_ID도 config에서 따른다.
    domain_id = (
        (cfg.get("bridge_local") or {}).get("ros", {}).get("domain_id")
        or cfg.get("ros", {}).get("domain_id")
    )
    if domain_id is not None and "ROS_DOMAIN_ID" not in os.environ:
        os.environ["ROS_DOMAIN_ID"] = str(domain_id)

    node = BridgeNode("gui_bridge")

    ws = WebSocketServer()
    tasks: list[asyncio.Task] = []
    if kind in ("websocket", "ws", "mock"):
        tasks.append(asyncio.create_task(ws.serve(host, port)))

    # rclpy가 있으면 ROS2 구독자 → WS broadcast를 연결한다. 그렇지 않으면
    # UI smoke 테스트 동안 GUI가 계속 렌더되도록 5Hz 합성 producer로
    # 되돌아간다.
    robots = _default_robots(cfg)
    if HAS_RCLPY and node.ros_node is not None:
        loop = asyncio.get_running_loop()

        def sink(env: dict) -> None:
            # ros 콜백은 rclpy executor 스레드에서 실행된다; 안전하게
            # broadcast하기 위해 asyncio 루프로 넘어간다.
            loop.call_soon_threadsafe(asyncio.create_task, ws.broadcast(env))

        TopicSubscriber(node, robots, sink)
        command_handler.set_instance(command_handler.CommandHandler(node, robots))
    else:
        tasks.append(asyncio.create_task(ws.producer_loop()))

    if kind == "grpc":
        tasks.append(asyncio.create_task(grpc_serve(host, port)))

    if HAS_RCLPY:
        # 단일 스레드 executor라 spin_once는 틱마다 ready 콜백 하나만 처리한다.
        # TF listener(/tf 고빈도) + odom/scan + 20Hz TF 타이머가 함께 돌면 틱당
        # 하나로는 적체돼 pose 갱신이 밀린다. 틱마다 여러 개를 드레인하고 슬립을
        # 줄여 처리량 천장을 높인다(유휴 시 spin_once는 즉시 반환해 저비용).
        async def spin_loop() -> None:
            while True:
                for _ in range(8):
                    node.spin_once()
                await asyncio.sleep(0.005)
        tasks.append(asyncio.create_task(spin_loop()))

    try:
        await asyncio.gather(*tasks)
    finally:
        node.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="../backend/configs/config.yaml")
    args = parser.parse_args()
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
