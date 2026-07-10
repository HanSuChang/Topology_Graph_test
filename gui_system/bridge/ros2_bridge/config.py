"""Python 브릿지용 config 로더.

Go 백엔드가 읽는 동일한 YAML(backend/configs/config.yaml)과 브릿지
로컬 config.yaml을 로드한다. 진실 원천은 Go 측이 소유하며, 브릿지
config은 Python 프로세스만 신경 쓰는 항목(카메라 디바이스 경로, ROS2
namespace)을 위해 존재한다.
"""
from __future__ import annotations

import os
from typing import Any

import yaml


def load(path: str) -> dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def bridge_address(cfg: dict[str, Any]) -> tuple[str, int]:
    """`bridge.address`를 합리적 기본값과 함께 (host, port)로 파싱한다."""
    addr = cfg.get("bridge", {}).get("address", "0.0.0.0:9090")
    host, _, port = addr.partition(":")
    return host or "0.0.0.0", int(port or "9090")
