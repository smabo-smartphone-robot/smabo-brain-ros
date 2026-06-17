# smabo-brain-ros

smabo-brain の **ROS 2 ラッパー**。smabo-brain の純ロジック（`brain.odometry` など）を
**import して再利用**し、ROS ノードとして公開します。積分などのアルゴリズムは smabo-brain
側の1か所に集約し、こちらは ROS 時刻・メッセージ・tf への変換だけを担います。

> 役割分担: `brain/odometry.py`（純 core、値を返す）↔ 各トランスポート（WS=smabo-brain /
> ROS=このリポジトリ）が時刻・msg・tf を付与。WebSocket 版は smabo-brain の `relay.py`。

## パッケージ構成

| パッケージ | ビルドタイプ | 内容 |
|-----------|------------|------|
| `smabo_interfaces` | ament_cmake | カスタムメッセージ（`WheelVel.msg` = left/right/dt） |
| `smabo_brain_ros`  | ament_python | `odom_node`（`/wheel_vel` → `/odom` + tf、brain の core を使用） |

## 前提

- ROS 2（Humble 以降を想定）
- **smabo-brain が import 可能**であること（`brain` パッケージ）。次のいずれか:
  ```bash
  # 開発時はリポジトリroot を PYTHONPATH に通すのが確実
  export PYTHONPATH="$HOME/github/smabo-repos/smabo-brain:$PYTHONPATH"
  # もしくは pip で
  pip install -e ~/github/smabo-repos/smabo-brain
  ```

## ビルド & 実行（スクリプト）

```bash
./build.sh        # ワークスペース作成＋リンク＋colcon build
./run.sh          # ROS/WS を source、brain を PYTHONPATH に通して odom を launch
./run.sh wheel_separation:=0.20   # 追加引数は ros2 launch に渡る
```

環境変数で調整できます（既定値）:
`ROS_DISTRO`(humble) / `SMABO_WS`(~/smabo_ws) / `SMABO_BRAIN_DIR`(../smabo-brain)。
`run.sh` は未ビルドなら自動で `build.sh` を呼びます。

## ビルド & 実行（手動）

このリポジトリを colcon ワークスペースの `src/` に置いてビルドします。

```bash
mkdir -p ~/smabo_ws/src
ln -s ~/github/smabo-repos/smabo-brain-ros ~/smabo_ws/src/smabo-brain-ros
cd ~/smabo_ws
colcon build
source install/setup.bash

# brain を import できる状態で（PYTHONPATH / pip）
export PYTHONPATH="$HOME/github/smabo-repos/smabo-brain:$PYTHONPATH"

ros2 launch smabo_brain_ros odom.launch.py
# または
ros2 run smabo_brain_ros odom_node --ros-args -p wheel_separation:=0.20
```

### パラメータ（`odom_node`）

| 名前 | 既定 | 説明 |
|------|------|------|
| `wheel_separation` | 0.15 | トレッド幅 (m) |
| `odom_frame` / `base_frame` | `odom` / `base_link` | tf / header の frame |
| `publish_tf` | `true` | `odom→base_frame` の tf を出すか |
| `input_topic` / `output_topic` | `wheel_vel` / `odom` | 入出力トピック |
| `pose_xx`/`pose_yy`/`pose_aa`/`twist_vv`/`twist_ww` | 0.001 | 共分散対角 |

入力は `smabo_interfaces/msg/WheelVel`（`left`/`right` m/s, `dt` s）。
ESP32 からは rosbridge_suite 経由でこのトピックに publish する想定です。

## 設計メモ

- 積分ロジックは **smabo-brain の `brain.odometry.Odometry`** をそのまま使用（重複実装しない）。
- core は `time`・メッセージ形式・tf を持たない純関数。ROS 固有処理（ROS 時刻・nav_msgs・
  tf broadcast）はこのノード側に閉じる。
- 非 ROS クライアント（web / app / esp32）は rosbridge_suite で ROS グラフに接続する想定
  （詳細は smabo の設計ドキュメント参照）。
