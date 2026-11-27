# 春ロボ ROS 2 シミュレーション

Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。関東春ロボコン2027の競技フィールドで前後・左右・旋回を操作し、仮想 LiDAR で地図を作れる。

機体は仮寸法 0.55 × 0.42 m の箱型。Gazebo が機体座標系の速度指令で平面運動を直接再現する。車輪、モーター、CAN、スリップのモデル化は今後の作業。オドメトリは Gazebo の機体姿勢を使い、仮想 IMU は `/imu/data` に配信する。SLAM Toolbox の入力は LiDAR とオドメトリ。

## フィールドの再現範囲

[関東春ロボコン2027 公式フィールド図面](https://kantouharurobo.com/static/haru/2027/KantouHaruRobocon_2027_Field_1.00.pdf) の平面図を基に、外形 7.000 × 6.924 m、木枠の幅 38 mm・高さ 89 mm、両チームのスタートゾーンとベースゾーン、4つのトレンチと白線を配置した。ロボットは赤のスタートゾーンからコート中央方向を向いて開始する。木枠を地図に捉えるため、仮想 LiDAR の走査面は床上 65 mm とした。

サプライは赤・青それぞれ15個の固定球。キャンプとフロントには固定の円柱を使用し、かごの寸法は仮値。競技物の回収・運搬や得点判定は今後の作業。

## ビルドと起動

以下のコマンドはリポジトリのルートから実行する。Ubuntu 24.04 で Jazzy、colcon、初期化済みの rosdep を用意する。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths simulation/src --ignore-src -r -y
(cd simulation && colcon build --base-paths src)
source simulation/install/setup.bash
ros2 launch harurobo_sim sim.launch.py
```

以前の練習用フィールドは起動時に `field:=training.sdf` を指定すると選べる。GUI なしで起動する場合は `headless:=true` を付ける。仮想 LiDAR のレンダリングはヘッドレスでも有効。

## 端末から操作

別端末を開き、シミュレーション用の ROS ドメインを選択する。実機と区別するため、シミュレーションの launch はドメイン 87 とローカル探索を設定する。

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=87 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

各コマンドを順番に実行する。値の符号を反転すると、後退・右移動・時計回りを確認できる。

```bash
# 前進
ros2 topic pub -r 10 -t 20 /cmd_vel/manual geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.25}}}'

# 左への平行移動
ros2 topic pub -r 10 -t 20 /cmd_vel/manual geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {y: 0.25}}}'

# 反時計回りに旋回
ros2 topic pub -r 10 -t 20 /cmd_vel/manual geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {angular: {z: 0.4}}}'
```

### キーボードで操作する場合

別端末で上記の ROS ドメインを設定して実行する。必要なパッケージは `ros-jazzy-teleop-twist-keyboard`。

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p stamped:=true -p frame_id:=base_link -p speed:=0.25 -p turn:=0.4 \
  -r cmd_vel:=/cmd_vel/manual
```

`i` と `,` で前後、`j` と `l` で旋回、Shift を押しながら `j` と `l` で左右移動、`k` で停止する。
`q`、`w`、`e` は端末側の速度設定を上げる。シミュレーションが受け付ける並進速度は斜め移動も含めて合計 0.25 m/s、旋回速度は 0.50 rad/s まで。

`/odom` で位置と向きを確認できる。指令が 0.30 秒間届かないと、シミュレーション側のノードが速度ゼロを送る。

## コントローラーから操作

既存の `harurobo_teleop` を使用する。コントローラーを PC に接続し、シミュレーションを以下のコマンドで起動し直す。実機動作と同時にコントローラーを使う場合は、実機側で同じコントローラーの使用を停止する。

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths real/manual_drive/src/harurobo_teleop --ignore-src -r -y
(cd real/manual_drive && colcon build --base-paths src --packages-select harurobo_teleop)
source real/manual_drive/install/setup.bash
source simulation/install/setup.bash
ros2 launch harurobo_sim sim.launch.py teleop:=true
```

スティックを中立にして L1 を離し、中立のまま L1 を押し直す。左スティックが前後・左右、右スティック横が旋回。L1 を離すと停止する。端末から速度指令を試すときは `teleop:=false` で起動する。

## 地図を作成して保存

起動時に `slam:=true` を付けて機体を走らせる。`teleop:=true` や `headless:=true` と併用できる。SLAM Toolbox はシミュレーション時刻を使い、`/map` を配信する。

別端末で上記のドメイン設定を行い、`rviz2 --ros-args -p use_sim_time:=true` を起動すると、Fixed Frame を `map` にして Map 表示から `/map` を確認できる。

再編集可能なポーズグラフを保存する。出力先のディレクトリは先に作成する。

```bash
mkdir -p "$HOME/harurobo-sim-maps"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/harurobo-sim-maps/kanto_2027'}"
```

`kanto_2027.posegraph` と `kanto_2027.data` が生成される。占有格子を PGM / YAML で出力する場合は、別途 `ros-jazzy-nav2-map-server` を導入し、同じ端末で `ros2 run nav2_map_server map_saver_cli -f "$HOME/harurobo-sim-maps/kanto_2027"` を実行する。
