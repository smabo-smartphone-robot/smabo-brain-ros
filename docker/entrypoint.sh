#!/usr/bin/env bash
# Source ROS + the smabo workspace, ensure brain is importable, then exec the
# command (default: ros2 launch smabo_bringup bringup.launch.py).
set -e

source /opt/ros/humble/setup.bash

# If the workspace is mounted but not yet built (live-edit mode), build it.
if [ ! -f "${SMABO_WS}/install/setup.bash" ]; then
  echo "[entrypoint] workspace not built; building ..."
  cd "${SMABO_WS}"
  colcon build --symlink-install
fi
source "${SMABO_WS}/install/setup.bash"

# brain (smabo-brain pure logic) on PYTHONPATH as a fallback to the pip install.
export PYTHONPATH="${SMABO_BRAIN_DIR}:${PYTHONPATH:-}"

exec "$@"
