#!/usr/bin/env bash
# Install only the isolated physics runtime; never launch Isaac or physics.
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$task_dir/.." && pwd)"
venv_dir="${S300_NEWTON_VENV:-$task_dir/.newton-venv}"
python="$venv_dir/bin/python"
lock="$task_dir/newton-requirements.lock.txt"
mode="${1:-install}"
if [[ "$mode" == '--help' || "$mode" == '-h' ]]; then
  printf '%s\n' 'Usage: setup_s300_newton.sh [--check]' \
    'Installs pinned Newton 1.7 / MuJoCo+MJWarp 3.15 / Warp 1.17 into nvidia-sim/.newton-venv.' \
    '--check only validates installed package versions and dependencies; it does not start CUDA or Isaac.'
  exit 0
fi
if [[ "$mode" != 'install' && "$mode" != '--check' ]]; then
  printf 'Unknown setup mode: %s\n' "$mode" >&2
  exit 2
fi
export PYTHONNOUSERSITE=1
if [[ "$mode" == 'install' ]]; then
  if [[ ! -x "$python" ]]; then
    if [[ -n "${S300_NEWTON_BASE_PYTHON:-}" ]]; then
      base_python="$S300_NEWTON_BASE_PYTHON"
    elif command -v python3.12 >/dev/null 2>&1; then
      base_python="$(command -v python3.12)"
    elif [[ -x "$repo_dir/mujoco-benchmark/.mujoco315-render-venv/bin/python" ]]; then
      base_python="$repo_dir/mujoco-benchmark/.mujoco315-render-venv/bin/python"
    else
      printf '%s\n' 'Python 3.12 is required. Set S300_NEWTON_BASE_PYTHON to its executable.' >&2
      exit 1
    fi
    env -u PYTHONPATH "$base_python" -c \
      'import sys; assert sys.version_info[:2] == (3, 12), "Python 3.12 required"'
    env -u PYTHONPATH "$base_python" -m venv "$venv_dir"
  fi
  env -u PYTHONPATH "$python" -m pip install -r "$lock"
fi
if [[ ! -x "$python" ]]; then
  printf 'Missing Newton runtime: %s\nRun %s first.\n' "$python" "$0" >&2
  exit 1
fi
env -u PYTHONPATH "$python" - "$lock" <<'PY'
from importlib.metadata import version
from pathlib import Path
import json
import sys
if sys.version_info[:2] != (3, 12):
    raise SystemExit('Newton runtime must use Python 3.12')
installed = {}
for line in Path(sys.argv[1]).read_text().splitlines():
    line = line.strip()
    if not line or line.startswith('#'):
        continue
    name, expected = line.split('==')
    actual = version(name)
    if actual != expected:
        raise SystemExit(f'Runtime mismatch: {name} expected {expected}, installed {actual}')
    installed[name] = actual
print(json.dumps({'python': sys.executable, 'packages': installed,
                  'cuda_or_simulator_started': False}, indent=2))
PY
env -u PYTHONPATH "$python" -m pip check
printf '%s\n' 'Newton physics runtime is ready. Isaac Sim remains an independent display process.'
