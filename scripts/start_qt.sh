#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if pgrep -f '[p]ython.*-m teleop[.]qt' >/dev/null; then
  echo "L20 Qt is already running. Select its window in the taskbar."
  exit 0
fi
if ! ip link show can0 >/dev/null 2>&1; then
  echo "can0 is missing. Check USB-CAN attachment to WSL."
  exit 1
fi
ip link set can0 down
ip link set can0 type can bitrate 1000000 listen-only off
ip link set can0 up
trap 'ip link set can0 down; ip link set can0 type can bitrate 1000000 listen-only on; ip link set can0 up' EXIT
QT_QPA_PLATFORM=wayland .venv/bin/python -m teleop.qt
