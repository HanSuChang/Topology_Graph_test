#!/usr/bin/env bash
# Produce the production React bundle the Go backend serves from
# backend/configs/config.yaml (server.static_dir = ../frontend/dist).
set -euo pipefail
cd "$(dirname "$0")/../frontend"

if [ ! -d node_modules ]; then
  npm install --no-audit --no-fund
fi
npm run build
