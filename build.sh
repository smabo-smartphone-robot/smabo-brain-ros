#!/usr/bin/env bash
# smabo-brain-ros ビルドスクリプト
#   colcon ワークスペースを用意し、このリポジトリを src にリンクしてビルドする。
#
# 環境変数:
#   ROS_DISTRO   使用する ROS 2 ディストリ（既定: humble）
#   SMABO_WS     ワークスペース（既定: ~/smabo_ws）
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"

: "${ROS_DISTRO:=humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
[ -f "$ROS_SETUP" ] || { echo "ROS not found: $ROS_SETUP (set ROS_DISTRO)"; exit 1; }
# shellcheck disable=SC1090
source "$ROS_SETUP"

WS="${SMABO_WS:-$HOME/smabo_ws}"
mkdir -p "$WS/src"
ln -sfn "$HERE" "$WS/src/smabo-brain-ros"

cd "$WS"
colcon build --symlink-install "$@"
echo "[build.sh] done. -> source $WS/install/setup.bash"
