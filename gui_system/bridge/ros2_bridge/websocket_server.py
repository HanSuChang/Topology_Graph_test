"""Go config에서 `bridge.type == websocket`일 때 브릿지 트랜스포트로
쓰이는 WebSocket 서버. Must 데모는 이를 synth 모드로 실행해 ROS2 없이도
대시보드를 종단 간 검증할 수 있게 한다.
"""
from __future__ import annotations

import asyncio
import json
import math
import sys
import time
from typing import Any, Set

try:
    import websockets  # type: ignore
except ImportError:
    websockets = None  # type: ignore

from . import command_handler


class WebSocketServer:
    def __init__(self) -> None:
        self.clients: Set[Any] = set()

    async def broadcast(self, msg: dict[str, Any]) -> None:
        if not self.clients:
            return
        data = json.dumps(msg)
        await asyncio.gather(*[c.send(data) for c in list(self.clients)], return_exceptions=True)

    # path 인자는 구버전 websockets(<11)가 핸들러를 (websocket, path)로 호출하기
    # 때문에 필요. 신버전(>=11)은 (websocket)만 넘기므로 기본값으로 양쪽 모두 호환.
    async def _handler(self, websocket: Any, path: str = "") -> None:
        self.clients.add(websocket)
        try:
            async for raw in websocket:
                try:
                    cmd = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                result = command_handler.handle(cmd)
                reply = {
                    "type": "command_result",
                    "timestamp": int(time.time() * 1000),
                    "seq": 0,
                    "version": 1,
                    "request_id": cmd.get("request_id", ""),
                    "payload": result,
                }
                await websocket.send(json.dumps(reply))
        finally:
            self.clients.discard(websocket)

    async def producer_loop(self) -> None:
        """실제 ROS2 토픽이 흐르지 않을 때 대시보드가 렌더할 무언가를
        갖도록 세 로봇의 5Hz 원형 pose를 합성한다. Should 단계에서
        topic_subscriber로부터의 fan-out으로 대체된다."""
        ids = ["tb3_leader", "follower_1", "follower_2"]
        seq = 0
        start = time.monotonic()
        while True:
            now = time.monotonic() - start
            seq += 1
            for i, rid in enumerate(ids):
                phase = now * 0.4 + i * (2 * math.pi / 3)
                ts = int(time.time() * 1000)
                await self.broadcast({
                    "type": "robot_pose",
                    "robot_id": rid,
                    "timestamp": ts,
                    "seq": seq,
                    "version": 1,
                    "payload": {
                        "robot_id": rid,
                        "pose": {"x": math.cos(phase) * 1.5, "y": math.sin(phase) * 1.5, "theta": phase},
                        "battery": 0.85 - i * 0.05,
                        "status": "moving",
                        "connection_state": "online",
                    },
                })
                # synth 텔레메트리 — 원형 주행 속도(≈r*ω=0.6m/s)에 약간의 변동.
                # 실제 브릿지는 /odom에서 telemetry를 내며, 이는 ROS2 없이도
                # 분석 탭 속도 차트를 검증/시연할 수 있게 한다.
                await self.broadcast({
                    "type": "telemetry",
                    "robot_id": rid,
                    "timestamp": ts,
                    "seq": seq,
                    "version": 1,
                    "payload": {
                        "robot_id": rid,
                        "timestamp": ts,
                        "velocity": round(0.6 + 0.1 * math.sin(phase), 3),
                        "battery": 0.85 - i * 0.05,
                    },
                })
            await asyncio.sleep(0.2)

    async def serve(self, host: str, port: int) -> None:
        if websockets is None:
            print("websockets package not installed; install with: pip install websockets", file=sys.stderr)
            return
        async with websockets.serve(self._handler, host, port):  # type: ignore
            print(f"[bridge] ws listening on {host}:{port}", flush=True)
            await asyncio.Future()
