#!/bin/bash
# Same tool discovery and ELF/flash-byte preflight as the historical IMU script.
set -euo pipefail
case "${1:-}" in
  verify|probe) mode=$1 ;;
  inspect) mode=motor ;;
  *) echo 'Usage: scripts/debug-gimbal.sh verify|inspect|probe (3S DISCONNECTED)' >&2; exit 2 ;;
esac
[[ $# -eq 1 ]] || exit 2
exec "$(dirname "$0")/debug-onboard-imu.sh" "$mode"
