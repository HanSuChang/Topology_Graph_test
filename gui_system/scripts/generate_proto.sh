#!/usr/bin/env bash
# Regenerate the gRPC stubs from proto/robot_bridge.proto.
# Only run after editing the .proto. Output lands in
#   backend/internal/proto/  (Go)
#   bridge/ros2_bridge/proto/ (Python)
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v protoc >/dev/null 2>&1; then
  echo "protoc not installed. apt install protobuf-compiler" >&2
  exit 1
fi

OUT_GO=backend/internal/proto
OUT_PY=bridge/ros2_bridge/proto
mkdir -p "$OUT_GO" "$OUT_PY"

protoc -I proto \
  --go_out="$OUT_GO" --go_opt=paths=source_relative \
  --go-grpc_out="$OUT_GO" --go-grpc_opt=paths=source_relative \
  proto/robot_bridge.proto

python3 -m grpc_tools.protoc -I proto \
  --python_out="$OUT_PY" --grpc_python_out="$OUT_PY" \
  proto/robot_bridge.proto

echo "generated."
