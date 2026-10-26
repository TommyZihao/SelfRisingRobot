#!/bin/sh
set -eu
cd "$(dirname "$0")"
if [ ! -x .venv/bin/mjpython ]; then
  echo "请先运行 setup_viewer.sh 安装查看器依赖。"
  exit 1
fi
exec .venv/bin/mjpython live_robot_viewer.py "$@"
