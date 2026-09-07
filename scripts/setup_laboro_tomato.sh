#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${ROOT_DIR}/.venv-laboro"
MODEL_DIR="${ROOT_DIR}/models/laboro_tomato"
MODEL_PATH="${MODEL_DIR}/laboro_tomato_little_48ep.pth"
MODEL_URL="https://assets.laboro.ai.s3.amazonaws.com/laborotomato/laboro_tomato_little_48ep.pth"
MODEL_SHA256="694ca8a9606124ffe36f5316fb34170299c25333f64c2a1ece9e5cc8ac4b7ca6"

python3 -m venv "${VENV_DIR}"
"${VENV_DIR}/bin/python" -m pip install --upgrade pip wheel setuptools
"${VENV_DIR}/bin/pip" install \
  torch==2.1.2 torchvision==0.16.2 \
  --index-url https://download.pytorch.org/whl/cu121
"${VENV_DIR}/bin/pip" install \
  'numpy<2' 'mmengine>=0.10.3,<1.0' mmdet==3.3.0
"${VENV_DIR}/bin/pip" install mmcv==2.1.0 \
  -f https://download.openmmlab.com/mmcv/dist/cu121/torch2.1/index.html

mkdir -p "${MODEL_DIR}"
if [[ ! -f "${MODEL_PATH}" ]]; then
  # Laboro.AI's published S3 URL currently presents a certificate whose
  # hostname does not match the URL. Restrict --insecure to this official
  # model download and verify the exact checkpoint hash immediately after.
  curl --insecure --location --fail --retry 3 \
    --output "${MODEL_PATH}" "${MODEL_URL}"
fi
echo "${MODEL_SHA256}  ${MODEL_PATH}" | sha256sum --check

"${VENV_DIR}/bin/python" - <<'PY'
import mmcv
import mmdet
import mmengine
import torch

print("LaboroTomato inference environment ready")
print("torch", torch.__version__, "cuda", torch.cuda.is_available())
print("mmcv", mmcv.__version__)
print("mmengine", mmengine.__version__)
print("mmdet", mmdet.__version__)
PY
