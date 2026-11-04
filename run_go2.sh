#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
CONDA_BASE="${CONDA_BASE:-/data/anaconda3}"
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate isaacsim_510
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export GO2_DATA_DIR="${GO2_DATA_DIR:-$PWD/outputs/bridge}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export ISAACSIM_ACCEPT_EULA=Y
export PYTHONUTF8=1
launch_args=()
if [[ "${GO2_NO_RENDER:-0}" != 1 ]]; then
  launch_args+=(--enable_cameras)
fi
exec python isaac_go2_ros2.py "${launch_args[@]}" "$@"
