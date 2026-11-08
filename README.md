# smabo-brain-ros

smabo-brain の **ROS 2 ラッパー**であり、smabo の **ROS 2 ランタイム一式**。
smabo-brain の純ロジック（`brain.odometry`・`brain.topics` など）を **import して再利用**しつつ、
ROS 2 のエコシステム（**rosbridge_suite / Nav2 / MoveIt2**）でナビゲーションと動作計画を提供します。

> 設計方針: **ROS 2 通信はこのリポジトリ内部だけ**で使い、外部（smabo-web / smabo-app /
> smabo-esp32）とは **rosbridge_suite による WebSocket ↔ ROS 2 相互変換**で接続します。
> 積分などのアルゴリズムは smabo-brain 側の1か所に集約し、こちらは ROS 時刻・メッセージ・
> tf・アクションへの変換だけを担います（WebSocket 版の中継は smabo-brain の `server.py`）。

## パッケージ構成

| パッケージ | ビルドタイプ | 内容 |
|-----------|------------|------|
| `smabo_interfaces` | ament_cmake | カスタムメッセージ（`WheelVel.msg` = left/right/dt）とサービス（`GetFile.srv` = URDF / SRDF / メッシュの取得） |
| `smabo_brain_ros`  | ament_python | `odom_node`（`/wheel_vel`→`/odom`+tf）、`servo_trajectory_bridge`（MoveIt の FollowJointTrajectory→`/servo/command`）、`webrtc_camera_node`（smabo-app の WebRTC 映像を終端し `sensor_msgs/Image`＝`/camera/image_raw` を publish、`brain.webrtc_hub` 再利用）、`image_processor_node`＋`gaze`/`neck`/`drive`\_policy\_node（画像処理、`brain.vision` 再利用）、`speech_recognizer_node`（`/speech/audio`→STT→`/speech/recognized`、`brain.stt` 再利用）、`robot_model_server`（URDF / SRDF / メッシュを `/smabo/get_file` で rosbridge クライアントへ提供）、`relays.launch.py`（送信元 prefix 剥がし） |
| `smabo_description` | ament_cmake | URDF/xacro ロボットモデル。smabo-hardware の STEP から生成したメッシュ（`scripts/split_smabo_meshes.py`）。共通の `base_link` に arm / hand / breadboard / tt_motor / neck を xacro 引数で取り付け |
| `smabo_moveit_config` | ament_cmake | MoveIt2 設定（SRDF は xacro。`arm` グループ＝`arm_joint_1..3`（位置のみの IK）、`gripper`＝`arm_joint_4`、`head`＝`head_pan`（`mount:=neck` 時）） |
| `smabo_navigation` | ament_cmake | Nav2 設定（costmap は `/scan`、AMCL/SLAM、プレースホルダ地図）。`nav_manager`（SLAM↔AMCL 切替・地図保存）は `smabo_brain_ros` |
| `smabo_bringup` | ament_cmake | 一括起動 launch（rosbridge＋relays＋odom＋RSP＋Nav2＋MoveIt＋vision＋speech）と Docker 連携 |

> 現状は **ビルド可能な雛形（scaffold）**です。sim/単体で起動確認できる状態までを対象とし、
> 実機での地図作成・Nav2/MoveIt のゲイン調整は次のステップです。

## システム構成（トポロジ）

```
smabo-web / smabo-app / smabo-esp32
        │  WebSocket（rosbridge v2 JSON、:9090）
        ▼
  rosbridge_suite ───────────────── ROS 2 graph（このリポジトリ内）
        │
        ├─ relays              /web,/esp32,/app prefix を canonical 名へ
        ├─ odom_node           /wheel_vel → /odom (+tf)   ← brain.odometry を再利用
        ├─ speech_recognizer   /speech/audio → STT → /speech/recognized  ← brain.stt を再利用
        ├─ Nav2                /odom + /scan → /cmd_vel
        └─ MoveIt2             arm 計画 → servo_trajectory_bridge → /servo/command

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

# SLAM（地図作成）モードで起動
docker compose run --rm smabo-ros ros2 launch smabo_bringup bringup.launch.py slam:=true
```

SLAM ↔ ナビの切り替えは再起動不要で、smabo-web の Navigation タブから行えます（下記「地図作成からナビゲーションまで」）。

ホストネットワークで `:9090` を公開するので、smabo-web / ESP32 から
`ws://<このホスト>:9090` に接続できます。

### bringup の引数

| 引数 | 既定 | 説明 |
|---|---|---|
| `sim` | `false` | アームを疑似化（`servo_trajectory_bridge` が `/joint_states` を配信） |
| `slam` | `false` | 起動時のモード。`true` で slam_toolbox（地図作成）、`false` で map_server＋AMCL（`map` の地図でナビ）。起動後は Navigation タブで切り替え可 |
| `map` | プレースホルダ | 起動時にナビで使う地図（yaml のパス、または保存済み地図の名前） |
| `maps_dir` | `$SMABO_MAPS_DIR` → `~/smabo-maps` | 地図の保存先（Docker ではホストの `~/smabo-maps` をマウント） |
| `use_nav` / `use_moveit` / `use_vision` / `use_speech` / `use_rosbridge` | `true` | 各サブシステムの起動可否 |
| `rosbridge_port` | `9090` | rosbridge のポート |
| `arm` / `hand` / `breadboard` | `true` / `false` / `false` | ロボットの構成（URDF・SRDF の xacro 引数） |
| `mount` | `none` | `none` / `tt_motor`（移動台車） / `neck`（床置きのパン台） |

## ロボットモデル

`smabo_description/urdf/smabo.urdf.xacro` を xacro 引数で構成します（`description.launch.py`・
`move_group.launch.py`・`bringup.launch.py` も同じ引数を受け取ります）。

```bash
xacro smabo.urdf.xacro arm:=false hand:=true breadboard:=true mount:=neck
```

| モジュール | xacro | 主なリンク / 関節 |
|---|---|---|
| base（共通） | `smabo_base.xacro` | `base_link`（ベースの箱の底面中央、x＝前） |
| breadboard | `smabo_base.xacro` | `breadboard_link`（fixed） |
| arm | `smabo_arm.xacro` | `arm_base_link` → `arm_joint_1`〜`3` → `arm_tool0`、グリッパ `arm_joint_4`（−90° 閉〜＋90° 開） |
| hand | `smabo_hand.xacro` | `hand_mount_link`、`left_hand` / `right_hand`（＋で前へ上がる） |
| tt_motor | `smabo_tt_motor.xacro` | `chassis_link`、`left/right_wheel_joint`（半径 0.030 m、間隔 0.130 m）、`laser`（仮） |
| neck | `smabo_neck.xacro` | `pan_stand_link`（根）→ `head_pan` → `pan_link` → `base_link` |

SG90・MG996R・TT モータ・車輪・キャスターは STEP に無いため、簡易形状でメッシュに含めています。
メッシュの再生成は `python3 smabo_description/scripts/split_smabo_meshes.py smabo_description/meshes`。

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
いずれも `brain.vision` の同じ関数を使います。ただし smabo-brain の WS リレーとは次の点が
異なります。

- policy ノードは `/vision/config` の `behaviors`（look_at / servo / drive）を参照せず、
  `/vision/detections` を受けるたびにそれぞれ出力します。
- 首・走行の追従は PID の状態を持たず、ゲイン（`servo_gain`・`drive_k_ang`/`drive_k_lin`）による
  比例制御です。

| ノード | 入力 | 出力 |
|---|---|---|
| `webrtc_camera_node` | WebRTC 映像（smabo-app、シグナリングは `/webrtc/*`） | `sensor_msgs/Image`（`/camera/image_raw`、bgr8） |
| `image_processor_node` | `sensor_msgs/Image`（`/camera/image_raw`） | `vision_msgs/Detection2DArray`（`/vision/detections`）＋`std_msgs/String`（`/vision/markers`）＋`speak` 時 `/speech/say` |
| `speech_recognizer_node` | `audio_common_msgs/AudioData`（`/speech/audio`、WAV） | `std_msgs/String`（`/speech/recognized`、STT 結果） |
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

## 地図作成からナビゲーションまで

Nav2 のナビゲーションサーバ（planner / controller / bt_navigator）は常時起動し、
`map` フレームと `/map` を出す部分だけを `nav_manager` ノードが子プロセスとして切り替えます。

| モード | 起動するもの | `map → odom` / `/map` |
|---|---|---|
| `mapping` | slam_toolbox（online async） | SLAM が生成 |
| `navigation` | nav2_bringup `localization_launch.py`（map_server＋AMCL） | 保存済み地図と AMCL |
| `off` | なし | なし（`/odom` のみで手動走行） |

smabo-web の Navigation タブ「Map」から次の順に操作します。

1. **Start SLAM** → Drive タブなどでロボットを走らせて地図を作る（3D ビューに `/map` が育つ）
2. 名前を付けて **Save** → `maps_dir/<名前>.yaml` と `.pgm`（`nav2_map_server map_saver_cli`）
3. 保存済み地図を選んで **Use** → map_server＋AMCL に切り替え。直前まで SLAM で作っていた地図なら、
   SLAM 上のロボット位置を `/initialpose` として AMCL に引き継ぐので、そのままゴールを指定できます
   （それ以外は Initial Pose で位置を合わせてから）

プロトコル（rosbridge 経由。smabo-brain 側では ROS 専用トピックとして転送しません）:

| トピック | 型 | 内容 |
|---|---|---|
| `/web/nav/command` → `/nav/command` | `std_msgs/String`（JSON） | `{"op": "mapping"}` / `{"op": "save", "name": "room_1"}` / `{"op": "navigate", "name": "room_1"}` / `{"op": "stop"}` |
| `/nav/state` | `std_msgs/String`（JSON, transient local, 1 秒ごと） | `{mode, map, maps: [{name, saved, resolution}], busy, message, maps_dir}` |

地図名は英数字と `_ . -`（64 文字まで）。Docker では `docker-compose.yml` がホストの `~/smabo-maps` を
`/opt/smabo_maps`（`SMABO_MAPS_DIR`）にマウントするので、コンテナを作り直しても地図は残ります。

## smabo-web からの操作（3D ビューア）

smabo-web に **Navigation**・**Motion Plan** タブを追加しました（roslibjs＋ros3djs）。

- **Navigation**: `/map`・`/scan`・ロボット URDF を 3D 表示し、`/initialpose` の設定と
  `navigate_to_pose` アクションによる目標指定（Nav2 の RViz 相当）。SLAM → 地図保存 → 保存地図でナビの切り替え。
- **Motion Plan**: MoveIt の MotionPlanning パネル相当。URDF・SRDF・メッシュを
  `robot_model_server`（`/smabo/get_file`）から rosbridge 経由で取得して three.js で描画し、
  現在の姿勢（`/joint_states`）・目標姿勢（半透明）・計画した軌道（アニメーション）を表示。
  目標は関節スライダー・SRDF の登録姿勢・アーム先端のドラッグ（`/compute_ik` による位置のみの IK）で
  指定し、`/plan_kinematic_path` で計画、`/execute_trajectory` で実行します。rosbridge が
  アクションに対応していない場合は、計画した軌道を `/servo/command` へ直接送れます。

ヘッダの brain ホスト（既定 `smabo-brain.local:9090`）に rosbridge のアドレスを入れて、
各タブの「Connect ROS」で接続します。

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
ros2 action list    # /smabo_arm_controller/follow_joint_trajectory, /smabo_gripper_controller/follow_joint_trajectory, /move_action, /execute_trajectory
ros2 service list | grep -E 'get_file|compute_ik|plan_kinematic_path'
ros2 topic echo /servo/command            # Motion Plan の Execute で出力

# 4) web
cd ../smabo-web && npm install && npm run dev   # Navigation / Motion Plan タブ
```

## 設計メモ

- 積分ロジックは **smabo-brain の `brain.odometry.Odometry`** をそのまま使用（重複実装しない）。
- 送信元 prefix の定義は **`brain.topics`** に集約し、ここの `relays.launch.py` が
  `topic_tools relay` で同じ対応を ROS 側に展開（twist_mux/remap 移行の素地）。
- `servo_trajectory_bridge` は MoveIt の `FollowJointTrajectory`（arm・gripper の 2 アクション）を
  `/servo/command`（`trajectory_msgs/JointTrajectory`）へ転送するだけ。時間追従は ESP32 側が行う。
  `simulate:=true` のときのみ `/joint_states` を疑似配信。
- ロボットモデル（`smabo_description`）は smabo-esp32 の関節名・角度範囲・車輪ジオメトリに整合
  （`arm_joint_1..4`・`head_pan`・`left/right_hand`、`wheel_separation=0.13`）。
- ESP32 側（smabo-esp32）は `brain.rosbridge:true` で rosbridge に接続し、接続時に
  送信トピックを advertise・受信トピックを subscribe します。`/scan` は LD06 を UART 直結し
  `modes.lidar` で配信（`lidar_ld06.py`）。詳細は smabo-esp32 の README を参照。
