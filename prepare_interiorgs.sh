#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source "${CONDA_BASE:-/data/anaconda3}/etc/profile.d/conda.sh"
conda activate isaacsim_510
if [[ ! -d outputs/usd_tools/pxr ]]; then
  python -m pip install --no-deps --target outputs/usd_tools usd-core==26.11
fi
PYTHONPATH="$PWD/outputs/usd_tools" python scripts/inspect_interiorgs.py
