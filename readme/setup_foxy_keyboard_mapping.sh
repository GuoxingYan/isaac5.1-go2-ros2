#!/usr/bin/env bash
# Source this file after entering /home/foxy_box with sudo chroot.

_foxy_setup_error() {
  printf 'Foxy environment setup failed: %s\n' "$1" >&2
  return 1
}

source /opt/ros/foxy/setup.bash || { _foxy_setup_error /opt/ros/foxy/setup.bash; return 1 2>/dev/null || exit 1; }
source /root/fastlivo_ws/install/setup.bash || { _foxy_setup_error /root/fastlivo_ws/install/setup.bash; return 1 2>/dev/null || exit 1; }
source /root/GO2_fastlivo2/install/setup.bash || { _foxy_setup_error /root/GO2_fastlivo2/install/setup.bash; return 1 2>/dev/null || exit 1; }

if [[ ! -r /mnt/go2-live/src/ros2/go2_file_bridge_wsl2.py ]]; then
  _foxy_setup_error 'repository is not mounted at /mnt/go2-live/src'
  return 1 2>/dev/null || exit 1
fi
if [[ ! -r /mnt/go2-live/data/data.json ]]; then
  _foxy_setup_error 'simulation data is not mounted at /mnt/go2-live/data'
  return 1 2>/dev/null || exit 1
fi

export ROS_DOMAIN_ID=117
export ROS_LOCALHOST_ONLY=1
export GO2_DATA_DIR=/mnt/go2-live/data
export PYTHONPATH="/mnt/go2-live/src/ros2:${PYTHONPATH:-}"

printf 'Foxy environment ready: ROS_DISTRO=%s ROS_DOMAIN_ID=%s GO2_DATA_DIR=%s\n' \
  "${ROS_DISTRO:-unset}" "$ROS_DOMAIN_ID" "$GO2_DATA_DIR"

unset -f _foxy_setup_error
