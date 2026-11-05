#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
export CUDA_HOME="${GO2_GS_CUDA_HOME:-/usr/local/cuda-12.4}"
export TORCH_CUDA_ARCH_LIST=8.6
export MAX_JOBS=4
export GO2_DATA_DIR="${GO2_DATA_DIR:-$PWD/outputs/interiorgs-bridge}"
exec ./run_go2.sh --policy flat env_name=interiorgs sensor.semantic_segmentation=False "$@"
