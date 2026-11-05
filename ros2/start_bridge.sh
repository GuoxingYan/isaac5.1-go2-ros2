#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export GO2_DATA_DIR="${GO2_DATA_DIR:-$PWD/outputs/our-go2-bridge}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
FOXY_CHROOT="${GO2_FOXY_CHROOT:-/home/foxy_box}"
FOXY_PROJECT="${GO2_FOXY_PROJECT:-/root/isaac5.1-go2-ros2}"
FOXY_DATA_DIR="${GO2_FOXY_DATA_DIR:-$FOXY_PROJECT/outputs/$(basename "$GO2_DATA_DIR")}"
if [[ ! -x "$FOXY_CHROOT/bin/bash" ]]; then echo "Foxy chroot not found: $FOXY_CHROOT" >&2; exit 1; fi
run_foxy() { sudo chroot "$FOXY_CHROOT" /bin/bash -lc "source /opt/ros/foxy/setup.bash && source /root/fastlivo_ws/install/setup.bash && source /root/GO2_fastlivo2/install/setup.bash && cd '$FOXY_PROJECT' && export ROS_DOMAIN_ID='$ROS_DOMAIN_ID' GO2_DATA_DIR='$FOXY_DATA_DIR' && exec $*"; }
case "${1:-}" in
  --rviz)
    run_foxy python3 ros2/go2_file_bridge_wsl2.py &
    bridge_pid=$!
    trap 'kill "$bridge_pid" 2>/dev/null || true' EXIT INT TERM
    rviz2 -d "$PWD/rviz/go2.rviz" --ros-args -p use_sim_time:=true
    ;;
  --rviz-only)
    run_foxy rviz2 -d "$FOXY_PROJECT/rviz/go2.rviz" --ros-args -p use_sim_time:=true
    ;;
  *)
    run_foxy python3 ros2/go2_file_bridge_wsl2.py "$@"
    ;;
esac
