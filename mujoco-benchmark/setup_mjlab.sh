#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3.11 -m venv "${task_dir}/.mjlab-venv"
"${task_dir}/.mjlab-venv/bin/python" -m pip install --extra-index-url https://download.pytorch.org/whl/cu128 -r "${task_dir}/mjlab-requirements.lock.txt"
"${task_dir}/.mjlab-venv/bin/python" "${task_dir}/scripts/prepare_mjlab_model.py"
