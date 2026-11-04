#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export GO2_DATA_DIR="${GO2_DATA_DIR:-$PWD/outputs/bridge}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
elif [[ -f /data/apps/ros2-humble/activate.sh ]]; then
  source /data/apps/ros2-humble/activate.sh
else
  echo 'ROS2 Humble setup not found' >&2
  exit 1
fi
case "${1:-}" in
  --rviz)
    /usr/bin/python3 ros2/go2_file_bridge_wsl2.py &
    bridge_pid=$!
    trap 'kill "$bridge_pid" 2>/dev/null || true' EXIT INT TERM
    rviz2 -d "$PWD/rviz/go2.rviz"
    ;;
  --rviz-only)
    exec rviz2 -d "$PWD/rviz/go2.rviz"
    ;;
  *)
    exec /usr/bin/python3 ros2/go2_file_bridge_wsl2.py "$@"
    ;;
esac
