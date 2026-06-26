# smabo-brain-ros

smabo-brain の **ROS 2 ラッパー**であり、smabo の **ROS 2 ランタイム一式**。
smabo-brain の純ロジック（`brain.odometry`・`brain.topics` など）を **import して再利用**しつつ、
ROS 2 のエコシステム（**rosbridge_suite / Nav2 / MoveIt2**）でナビゲーションと動作計画を提供します。

> 設計方針: **ROS 2 通信はこのリポジトリ内部だけ**で使い、外部（smabo-web / smabo-app /
> smabo-esp32）とは **rosbridge_suite による WebSocket ↔ ROS 2 相互変換**で接続します。
> 積分などのアルゴリズムは smabo-brain 側の1か所に集約し、こちらは ROS 時刻・メッセージ・
> tf・アクションへの変換だけを担います（WebSocket 版の中継は smabo-brain の `relay.py`）。

## パッケージ構成

| パッケージ | ビルドタイプ | 内容 |
|-----------|------------|------|
| `smabo_interfaces` | ament_cmake | カスタムメッセージ（`WheelVel.msg` = left/right/dt） |
| `smabo_brain_ros`  | ament_python | `odom_node`（`/wheel_vel`→`/odom`+tf）、`servo_trajectory_bridge`（MoveIt の FollowJointTrajectory→`/servo/command`）、`webrtc_camera_node`（smabo-app の WebRTC 映像を終端し `sensor_msgs/Image`＝`/camera/image_raw` を publish、`brain.webrtc_hub` 再利用）、`image_processor_node`＋`gaze`/`neck`/`drive`\_policy\_node（画像処理、`brain.vision` 再利用）、`relays.launch.py`（送信元 prefix 剥がし） |
| `smabo_description` | ament_cmake | URDF/xacro ロボットモデル（差動二輪＋LD06＋4自由度アーム。esp32 config に整合） |
| `smabo_moveit_config` | ament_cmake | MoveIt2 設定（`arm` グループ＝`arm_joint_1..4`、`head`＝`head_pan`） |
| `smabo_navigation` | ament_cmake | Nav2 設定（costmap は `/scan`、AMCL/SLAM、プレースホルダ地図） |
| `smabo_bringup` | ament_cmake | 一括起動 launch（rosbridge＋relays＋odom＋RSP＋Nav2＋MoveIt）と Docker 連携 |

> 現状は **ビルド可能な雛形（scaffold）**です。sim/単体で起動確認できる状態までを対象とし、
> 実機での地図作成・Nav2/MoveIt のゲイン調整・メッシュ精密化は次のステップです。

## システム構成（トポロジ）

```
smabo-web / smabo-app / smabo-esp32
        │  WebSocket（rosbridge v2 JSON、:9090）
        ▼
  rosbridge_suite ───────────────── ROS 2 graph（このリポジトリ内）
        │
        ├─ relays           /web,/esp32,/app prefix を canonical 名へ
        ├─ odom_node        /wheel_vel → /odom (+tf)   ← brain.odometry を再利用
        ├─ Nav2             /odom + /scan → /cmd_vel
        └─ MoveIt2          arm 計画 → servo_trajectory_bridge → /servo/command

  smabo-app ══[WebRTC 映像, P2P]══> webrtc_camera_node ──[/camera/image_raw]──> image_processor → /vision/detections
        （シグナリング /webrtc/* のみ rosbridge 経由）
```

> **カメラ映像は rosbridge をバイパス**します。rosbridge は JSON over WebSocket のため、映像を
> base64 JSON（`/camera/image/compressed`）で通すとリアルタイム性が落ちます。`webrtc_camera_node`
> が WebRTC ピアとして映像を終端し、フレームをネイティブ `sensor_msgs/Image`（`/camera/image_raw`）
> として DDS に流すため、ROS グラフ内は高速です。シグナリング（`/webrtc/offer`/`answer`/ICE、
> `/webrtc/preview` 等）だけが小さい JSON として rosbridge を通ります。`brain.webrtc_hub` を
> smabo-brain（WS 版）と共有し、フレーム処理コールバックだけ差し替えています。

rosbridge の既定ポートは **9090** で、従来の smabo-brain リレーと同じです（差し替え可能）。

### トピック対応表

| 送信元（prefix 付き） | canonical | 処理 |
|---|---|---|
| `/esp32/wheel_vel` | `/wheel_vel` | `odom_node` が積分 → `/odom` |
| `/esp32/scan` | `/scan` | Nav2 costmap・SLAM の入力（LD06 はマイコン直結） |
| `/esp32/joint_states` | `/joint_states` | RSP / MoveIt の現在状態 |
| `/web/cmd_vel` | `/cmd_vel` | 手動テレオペ（Nav2 も `/cmd_vel` を出力） |
| `/web/initialpose` | `/initialpose` | AMCL の初期姿勢 |
| `/web/goal_pose` | `/goal_pose` | （参考。実ナビは `navigate_to_pose` アクション） |
| `/web/servo/command` | `/servo/command` | サーボ直接指令（trajectory_msgs/JointTrajectory） |

出力トピック（`/odom`・`/scan`・`/map`・`/plan` 等）は canonical 名のまま購読します
（prefix は付けません）。prefix 文字列は smabo-brain の `brain.topics.SOURCE_PREFIXES` を
import して単一の正としています。

## Docker で起動（推奨）

ROS 2 Humble・Nav2・MoveIt2・rosbridge を含む環境を Docker で用意します。
`smabo-brain` と `smabo-brain-ros` を **兄弟ディレクトリ**に置いてください
（`smabo-brain` はビルド時ではなく実行時にボリュームマウントされ、entrypoint が
`PYTHONPATH` に通します。`docker-compose.yml` の `../smabo-brain` 参照を参照）。

```bash
cd ~/github/smabo-repos/smabo-brain-ros
docker compose build
docker compose up           # 既定で bringup.launch.py を起動
```

別の構成で起動する場合:

```bash
# sim（実機なし。MoveIt 用に servo_trajectory_bridge が /joint_states を疑似配信）
docker compose run --rm smabo-ros ros2 launch smabo_bringup bringup.launch.py sim:=true

# SLAM で地図作成しながらナビ
docker compose run --rm smabo-ros ros2 launch smabo_bringup bringup.launch.py slam:=true
```

ホストネットワークで `:9090` を公開するので、smabo-web / ESP32 から
`ws://<このホスト>:9090` に接続できます。

### bringup の引数

| 引数 | 既定 | 説明 |
|---|---|---|
| `sim` | `false` | アームを疑似化（`servo_trajectory_bridge` が `/joint_states` を配信） |
| `slam` | `false` | AMCL+地図の代わりに slam_toolbox で地図作成 |
| `use_nav` / `use_moveit` / `use_rosbridge` | `true` | 各サブシステムの起動可否 |
| `rosbridge_port` | `9090` | rosbridge のポート |

## ローカル（ネイティブ ROS 2）で起動

ROS 2 Humble がインストール済みなら、付属スクリプトで odom 単体を起動できます
（従来どおり）。フルスタックは上記 Docker を推奨します。

```bash
./build.sh        # ワークスペース作成＋リンク＋colcon build（全パッケージ）
source ~/smabo_ws/install/setup.bash
export PYTHONPATH="$HOME/github/smabo-repos/smabo-brain:$PYTHONPATH"  # brain を import 可能に

ros2 launch smabo_bringup bringup.launch.py            # フル
ros2 launch smabo_brain_ros odom.launch.py             # odom 単体（従来）
ros2 launch smabo_brain_ros vision.launch.py mode:=aruco   # 画像処理一式
```

## 画像処理（vision）

`brain.vision`（smabo-brain の純ロジック）を ROS ノードでラップしたもの。検出は
`image_processor_node`、行動は gaze / neck / drive の各 policy ノードに分かれ、
いずれも `brain.vision` の同じ関数を使うので smabo-brain の WS リレーと挙動が一致
します。

| ノード | 入力 | 出力 |
|---|---|---|
| `webrtc_camera_node` | WebRTC 映像（smabo-app、シグナリングは `/webrtc/*`） | `sensor_msgs/Image`（`/camera/image_raw`、bgr8） |
| `image_processor_node` | `sensor_msgs/Image`（`/camera/image_raw`） | `vision_msgs/Detection2DArray`（`/vision/detections`）＋`std_msgs/String`（`/vision/markers`）＋`speak` 時 `/speech/say` |
| `gaze_policy_node`  | `/vision/detections` | `geometry_msgs/PoseStamped`（`/look_at`） |
| `neck_policy_node`  | `/vision/detections` | `trajectory_msgs/JointTrajectory`（`/servo/command`、既定 `head_pan` のみ） |
| `drive_policy_node` | `/vision/detections` | `geometry_msgs/Twist`（`/cmd_vel`、対象追従） |

設定は各ノードの **ROS 2 パラメータ**（`mode`/`color`/`hfov_deg`/`target_marker_id`/
`pan_joint`/`drive_*` など）で与えます。加えて全ノードが `/vision/config`
（`std_msgs/String` の `data` に JSON）を subscribe し、web/rosbridge から実行中に
部分上書きできます（smabo-brain と同じ `vision.merge_config` セマンティクス）。
`Detection2DArray` は画像サイズを持たないため、policy ノードは `image_width`/
`image_height` パラメータで正規化します（WS 経路では非標準ヒントを同梱）。

`odom_node` のパラメータは従来どおり（`wheel_separation`・`odom_frame`/`base_frame`・
`publish_tf`・`input_topic`/`output_topic`・共分散対角）。入力は `smabo_interfaces/msg/WheelVel`。

## smabo-web からの操作（3D ビューア）

smabo-web に **Navigation**・**Motion Plan** タブを追加しました（roslibjs＋ros3djs）。

- **Navigation**: `/map`・`/scan`・ロボット URDF を 3D 表示し、`/initialpose` の設定と
  `navigate_to_pose` アクションによる目標指定（Nav2 の RViz 相当）。
- **Motion Plan**: アーム URDF を 3D 表示し、関節目標から MoveIt の `move_action` で
  計画・実行、または `/servo/command` への直接指令（MoveIt の RViz 相当）。

ヘッダの brain ホスト（既定 `localhost:9090`）に rosbridge のアドレスを入れて
「Connect ROS」します。

> **TF と tf2_web_republisher**: ros3djs の `TFClient` は `tf2_web_republisher` を前提とします。
> 3D ビューでロボット・スキャンを正しい位置に描画するには ROS 側でこれを起動してください
> （Humble ではディストリにより別途導入が必要な場合があります）。未導入でもグリッド・地図・
> ロボットは原点基準で表示されます。
>
> **ros3djs ↔ three のバージョン整合**は既知の注意点です（`package.json` でピン留め）。

## 動作確認

```bash
# 1) ノード・トピックが上がっているか
ros2 node list      # rosbridge_websocket, odom, move_group, nav2 各種, servo_trajectory_bridge
ros2 topic list     # /odom, /scan, /cmd_vel, /joint_states, /map ...

# 2) sim で odom/costmap（別端末でダミー入力）
ros2 launch smabo_bringup bringup.launch.py sim:=true
ros2 topic pub -r 20 /esp32/wheel_vel smabo_interfaces/msg/WheelVel \
  '{left: 0.1, right: 0.1, dt: 0.05}'      # → /odom が動く
ros2 topic echo /odom

# 3) MoveIt 実行が /servo/command を出すか
ros2 action list    # /smabo_arm_controller/follow_joint_trajectory, /move_action
ros2 topic echo /servo/command            # Motion Plan の Plan&Execute で出力

# 4) web
cd ../smabo-web && npm install && npm run dev   # Navigation / Motion Plan タブ
```

## 設計メモ

- 積分ロジックは **smabo-brain の `brain.odometry.Odometry`** をそのまま使用（重複実装しない）。
- 送信元 prefix の定義は **`brain.topics`** に集約し、ここの `relays.launch.py` が
  `topic_tools relay` で同じ対応を ROS 側に展開（twist_mux/remap 移行の素地）。
- `servo_trajectory_bridge` は MoveIt の `FollowJointTrajectory` を `/servo/command`
  （`trajectory_msgs/JointTrajectory`）へ転送するだけ。時間追従は ESP32 側が行う
  （design.md §4-5）。`simulate:=true` のときのみ `/joint_states` を疑似配信。
- ロボットモデル（`smabo_description`）は smabo-esp32 の関節・車輪ジオメトリに整合
  （`arm_joint_1..4`・`head_pan`・`left/right_hand`、`wheel_separation=0.15`）。
- ESP32 側（smabo-esp32）は `brain.rosbridge:true` で rosbridge に接続し、接続時に
  送信トピックを advertise・受信トピックを subscribe します。`/scan` は LD06 を UART 直結し
  `modes.lidar` で配信（`lidar_ld06.py`）。詳細は smabo-esp32 の README を参照。
