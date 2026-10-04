#!/bin/sh
set -eu

cd "$(dirname "$0")"

if [ ! -x ".venv/bin/mjpython" ]; then
  python3 -m venv .venv
fi

if ! .venv/bin/python -c "import mujoco" >/dev/null 2>&1; then
  .venv/bin/python -m pip install -r requirements.txt
fi

exec .venv/bin/mjpython mujoco_sim.py
