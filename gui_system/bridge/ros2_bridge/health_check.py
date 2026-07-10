"""브릿지 측 health check.

Go 백엔드의 /api/v1/health는 브릿지에 대해 connected/disconnected 단일
플래그를 노출한다. 이 모듈은, 백엔드가 폴링할 수 있는 주기적 ping(또는
그 반대)을 추가하게 될 때 브릿지가 자신의 준비 상태를 노출하는 데 쓴다.
"""
from __future__ import annotations


def is_ready(has_rclpy: bool, ws_listening: bool) -> bool:
    return ws_listening and (has_rclpy or True)
