"""gRPC 서버 stub.

proto 계약이 확정되면 Should 단계에서 채워진다. Must 단계에서 브릿지는
WebSocket만 사용한다. 이 모듈은 __main__의 dispatcher가 gRPC 경로에서
ImportError 없이 `bridge.type`으로 분기할 수 있도록 존재한다.
"""
from __future__ import annotations

import asyncio


async def serve(host: str, port: int) -> None:
    print(f"[bridge] grpc server not implemented; would bind {host}:{port}", flush=True)
    await asyncio.Future()
