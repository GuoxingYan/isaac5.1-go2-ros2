#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
FOXY_CHROOT="${GO2_FOXY_CHROOT:-/home/foxy_box}"
FOXY_PROJECT="${GO2_FOXY_PROJECT:-/root/isaac5.1-go2-ros2}"
FOXY_REPORT_DIR="${GO2_FOXY_REPORT_DIR:-$FOXY_PROJECT/${GO2_REPORT_DIR:-reports/our-go2}}"
exec sudo chroot "$FOXY_CHROOT" /bin/bash -lc "source /opt/ros/foxy/setup.bash && source /root/fastlivo_ws/install/setup.bash && source /root/GO2_fastlivo2/install/setup.bash && cd '$FOXY_PROJECT' && export ROS_DOMAIN_ID='$ROS_DOMAIN_ID' GO2_REPORT_DIR='$FOXY_REPORT_DIR' GO2_PROBE_SECONDS='${GO2_PROBE_SECONDS:-600}' && exec python3 ros2/verify_topics.py"
