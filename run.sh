#!/usr/bin/env bash
# smabo-brain-ros 起動スクリプト
#   ROS 2 環境とワークスペースを source し、smabo-brain を import 可能にして
#   odom ノードを launch する。未ビルドなら build.sh を自動実行する。
#   追加引数はそのまま `ros2 launch` に渡る。
#
# 環境変数:
#   ROS_DISTRO        使用する ROS 2 ディストリ（既定: humble）
#   SMABO_WS          ワークスペース（既定: ~/smabo_ws）
#   SMABO_BRAIN_DIR   smabo-brain リポジトリの場所（既定: ../smabo-brain）
#
# 例:
#   ./run.sh
#   ./run.sh wheel_separation:=0.20
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

# 1. ROS 2 環境
: "${ROS_DISTRO:=humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "$ROS_SETUP" ] || { echo "ROS not found: $ROS_SETUP (set ROS_DISTRO)"; exit 1; }
# shellcheck disable=SC1090
source "$ROS_SETUP"

# 2. smabo-brain を import 可能に（brain.odometry を再利用）
BRAIN_DIR="${SMABO_BRAIN_DIR:-$(cd "$HERE/../smabo-brain" 2>/dev/null && pwd || true)}"
if [ -z "${BRAIN_DIR:-}" ] || [ ! -d "$BRAIN_DIR/brain" ]; then
  echo "smabo-brain が見つかりません。SMABO_BRAIN_DIR を設定してください。"; exit 1
fi
export PYTHONPATH="${BRAIN_DIR}:${PYTHONPATH:-}"

# 3. ワークスペース（未ビルドなら build）
WS="${SMABO_WS:-$HOME/smabo_ws}"
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "[run.sh] workspace not built; building ..."
  "$HERE/build.sh"
fi
# shellcheck disable=SC1090
source "$WS/install/setup.bash"

# 4. 起動
exec ros2 launch smabo_brain_ros odom.launch.py "$@"
