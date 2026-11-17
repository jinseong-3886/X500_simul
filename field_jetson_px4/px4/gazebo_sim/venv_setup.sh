#!/bin/bash
# Python env for the Flytrap PX4 Gazebo sim (WSL). Shares system site-packages for gz python bindings.
set -e
mkdir -p ~/flytrap_sim
cd ~/flytrap_sim
[ -d venv ] || python3 -m venv --system-site-packages venv
. venv/bin/activate
pip install -q --upgrade pip
pip install -q torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install -q ultralytics pymavlink pyulog
python -c "import torch, ultralytics, cv2, numpy; from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image; print('OK torch', torch.__version__, '| ultralytics', ultralytics.__version__, '| cv2', cv2.__version__, '| numpy', numpy.__version__, '| gz bindings OK')"
