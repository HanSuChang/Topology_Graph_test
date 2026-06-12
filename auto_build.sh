#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_NAME="${1:-amr_topology}"
WATCH_DIR="${ROOT_DIR}/src/${PACKAGE_NAME}"

if [[ ! -d "${WATCH_DIR}" ]]; then
  echo "watch directory not found: ${WATCH_DIR}" >&2
  exit 1
fi

cd "${ROOT_DIR}" || exit 1

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

if [[ -f "${ROOT_DIR}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  set +u
  source "${ROOT_DIR}/install/setup.bash"
  set -u
fi

build_once() {
  echo
  echo "[$(date '+%H:%M:%S')] colcon build --packages-select ${PACKAGE_NAME}"
  colcon build --packages-select "${PACKAGE_NAME}"
}

snapshot() {
  find "${WATCH_DIR}" \
    -type f \( \
      -name '*.cpp' -o \
      -name '*.hpp' -o \
      -name '*.h' -o \
      -name 'CMakeLists.txt' -o \
      -name 'package.xml' -o \
      -name '*.yaml' -o \
      -name '*.launch.py' \
    \) \
    -printf '%T@ %p\n' | sort
}

build_once

if command -v inotifywait >/dev/null 2>&1; then
  echo "Watching ${WATCH_DIR} with inotifywait"
  while true; do
    inotifywait -r -e close_write,move,create,delete \
      --include '.*\.(cpp|hpp|h|yaml|launch\.py)$|.*/CMakeLists\.txt$|.*/package\.xml$' \
      "${WATCH_DIR}" >/dev/null 2>&1
    build_once
  done
fi

echo "inotifywait not found; watching ${WATCH_DIR} by polling every 2 seconds"
last_snapshot="$(snapshot)"
while true; do
  sleep 2
  current_snapshot="$(snapshot)"
  if [[ "${current_snapshot}" != "${last_snapshot}" ]]; then
    last_snapshot="${current_snapshot}"
    build_once
  fi
done
