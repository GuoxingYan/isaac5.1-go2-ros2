#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p reports/scenes
for scene in obstacle-sparse obstacle-medium obstacle-dense warehouse warehouse-forklifts warehouse-shelves full-warehouse; do
  echo "[SCENE] $scene"
  GO2_NO_RENDER=1 GO2_DATA_DIR="$PWD/outputs/scene-bridge" timeout -k 5 150 ./run_go2.sh --headless --max-steps 30 --report-dir "reports/scenes/$scene" sensor.enable_lidar=False sensor.enable_camera=False sensor.enable_imu=False "env_name=$scene" > "reports/scenes/$scene.log" 2>&1
  echo "[SCENE EXIT] $scene $?"
done
for policy in flat rough; do
  echo "[MULTI] $policy"
  GO2_NO_RENDER=1 GO2_DATA_DIR="$PWD/outputs/scene-bridge" timeout -k 5 120 ./run_go2.sh --headless --max-steps 100 --policy "$policy" --report-dir "reports/scenes/multi-$policy" sensor.enable_lidar=False sensor.enable_camera=False sensor.enable_imu=False num_envs=3 > "reports/scenes/multi-$policy.log" 2>&1
  echo "[MULTI EXIT] $policy $?"
done
