#!/usr/bin/env bash
# Build and run the Go backend in dev mode against the default config.
# Reads the same paths the production binary uses, so configs/ here is
# the single source of truth.
set -euo pipefail
cd "$(dirname "$0")/.."

export PATH="$HOME/.local/go/bin:$PATH"

cd backend
go build -buildvcs=false -o gui_main ./cmd/gui_main
exec ./gui_main --config ./configs/config.yaml
