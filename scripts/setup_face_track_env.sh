#!/usr/bin/env bash
# F183：准备人脸检测运行环境（隔离 venv + mediapipe + 模型）。
#
# 为什么必须隔离：本仓 a3_shell_env.sh 把 /usr/lib/python3/dist-packages 放在 PYTHONPATH
# 前面，系统 pyparsing 2.4.7 会让 mediapipe 的 matplotlib 导入链失败；且 mediapipe 会
# 覆盖 numpy/opencv 版本，不能污染系统环境（LL-156/LL-168）。
#
# 用法：
#   ./scripts/setup_face_track_env.sh            # 首次安装（已存在则跳过）
#   ./scripts/setup_face_track_env.sh --force    # 重建 venv
#
# 装完约 730 MB（jaxlib 占大头），模型放 ~/.a3/models/face_landmarker.task。

set -euo pipefail

VENV="${HOME}/.a3/venv/face_track"
MODEL_DIR="${HOME}/.a3/models"
MODEL="${MODEL_DIR}/face_landmarker.task"
MODEL_URL="https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task"
MP_VERSION="0.10.18"

FORCE=0
[ "${1:-}" = "--force" ] && FORCE=1

mkdir -p "${MODEL_DIR}"

if [ "${FORCE}" = "1" ] || [ ! -x "${VENV}/bin/python" ]; then
  echo "[*] 创建 venv：${VENV}"
  rm -rf "${VENV}"
  python3 -m venv --system-site-packages "${VENV}"
  "${VENV}/bin/pip" install -q --upgrade pip
  echo "[*] 安装 mediapipe ${MP_VERSION}（约 730 MB，需要联网）"
  "${VENV}/bin/pip" install -q "mediapipe==${MP_VERSION}" "pyparsing>=3.1"
else
  echo "[+] venv 已存在：${VENV}（--force 可重建）"
fi

if [ "${FORCE}" = "1" ] || [ ! -s "${MODEL}" ]; then
  echo "[*] 下载 face_landmarker 模型"
  curl -L --fail --retry 3 -o "${MODEL}.tmp" "${MODEL_URL}"
  mv "${MODEL}.tmp" "${MODEL}"
else
  echo "[+] 模型已存在：${MODEL}"
fi

echo "[*] 自检（注意必须清掉本仓 PYTHONPATH 顺序）"
env -u PYTHONPATH "${VENV}/bin/python" - <<'PY'
import mediapipe, numpy, cv2
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
print(f"  mediapipe {mediapipe.__version__} | numpy {numpy.__version__} | cv2 {cv2.__version__}")
print(f"  FaceLandmarker API: {hasattr(vision, 'FaceLandmarker')}")
PY

cat <<EOF

[ok] 环境就绪。
     启动：ros2 launch a3_perception face_track.launch.py
     注意：检测节点必须用 ${VENV}/bin/python3 运行（launch 已处理）；
           手动跑时用 env -u PYTHONPATH 或把 ${VENV}/lib/python3.10/site-packages
           前置到 PYTHONPATH，否则 pyparsing 冲突。
EOF
