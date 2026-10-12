#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
render_env="$repo_dir/mujoco-benchmark/.mujoco315-render-venv"
base_python="${S300_RENDER_PYTHON:-python3.12}"
if [[ ! -x "$render_env/bin/python" ]]; then
  env -u PYTHONPATH "$base_python" -c \
    'import sys; assert sys.version_info >= (3,12), "PBR display requires Python >=3.12"'
  env -u PYTHONPATH "$base_python" -m venv "$render_env"
fi
env -u PYTHONPATH "$render_env/bin/python" -c \
  'import sys; assert sys.version_info >= (3,12), "Existing PBR environment requires Python >=3.12; recreate it"'
env -u PYTHONPATH "$render_env/bin/python" -m pip install \
  -r "$repo_dir/mujoco-benchmark/scripts/s300_smoke/requirements-render-lock.txt"
env -u PYTHONPATH "$render_env/bin/python" -c \
  'import sys,mujoco; print("Display Python:",sys.version.split()[0],"MuJoCo:",mujoco.__version__)'
