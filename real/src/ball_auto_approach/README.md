# カメラとLD03によるボールへの自動接近

ROS 2 Jazzyの実機ワークスペース用。停止目標はLD03からボール表面まで350mm（許容差±10mm）。ボールから350mmより離れた位置から開始する。既存のLD03ドライバ、SDLのPS4入力、4輪の較正済みミキサー、L1デッドマン、CAN watchdogを使う。自動接近は相対座標だけで動作する。

## ビルドと起動

```bash
cd ~/DM_IMU_ros2
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths real/src/ball_auto_approach real/src/ball_approach_interfaces --ignore-src -r -y
colcon build --base-paths real/src/ball_auto_approach real/src/ball_approach_interfaces --packages-up-to ball_auto_approach --symlink-install
source install/setup.bash
ros2 launch ball_auto_approach ball_auto_approach.launch.py
```

初期設定は外付けの Logitech Webcam C270 を使用する。USBの抜き差しで `/dev/videoN` が変わっても選択できるよう、`/dev/v4l/by-id/` のデバイス名を指定している。機体カメラのデバイスは `v4l2-ctl --list-devices` と `--list-formats-ext` で確認し、設定ファイルを機体用にコピーして `device` を指定する。`camera:=false` で既存の `/camera/image_raw` と `/camera/camera_info` を使用できる。起動引数はコマンド末尾に `--show-args` を付けて確認できる。

通常起動時、最終速度は `/cmd_vel_preview` に出る。既存の `/scan` を使用できる。LD03も同時に立ち上げる場合は `lidar:=true lidar_port:=/dev/ttyACM0` を追加し、正しいデバイス名とスキャン方向を確認する。すでに `/joy` が配信されている場合は `joy:=false` を追加する。

## カメラ認識だけを確認する

```bash
ros2 launch ball_auto_approach ball_camera.launch.py
```

カメラ、認識ノード、画像ビューアが起動する。色と輪郭の条件を満たすと、緑の輪郭、赤い中心点、中心座標と画像上の直径が表示される。初期設定は赤系のボール。表示先を `/ball/mask` に切り替えると、色抽出の状態を確認できる。画像認識は較正前から確認でき、方向ベクトルはカメラの較正後に配信する。GUIのない環境では `view:=false` を指定する。

## センサー設定

1. ROS camera calibrationで、実際に使う解像度とフォーカスでカメラを較正し、標準のカメラ較正YAMLの絶対パスを `usb_camera.calibration` に設定する。画像の解像度と一致した較正値があるときに方向を配信する。
2. `base_link` から `laser` と `camera_optical_frame` への実測TFを配信する。既存TFを使うか、`publish_tf:=true` にして `laser_xyz`、`laser_rpy`、`camera_xyz`、`camera_rpy` を実測値で指定する。位置はm、角度はrad、カメラの取り付け角度はX前方・Y左・Z上の `camera_link` 基準。起動引数の数値は設定例。レーザーの0°が前方を指し、左の物体がbase_linkの正のYになることをRVizで確認する。
3. 現用の直径約60mmのボールは半径0.030mを初期値とする。ボールの実測半径と中心高さを `ball_radius` と `ball_center_z` に設定する。レーザーの水平面がボールの中心近くを通るように取り付ける。機体の端と突出物まで含めて `half_length` と `half_width` を設定する。
4. `rviz:=true` でスキャン、推定球、検出画像を表示する。RVizのImage表示を追加して `/ball/mask` を開くとHSVマスクを確認できる。QoSはBest Effort。初期HSVは外付けカメラで確認した赤系のボール用。Hは0〜179、S/Vは0〜255。赤のようにHが端をまたぐ場合は下限Hを上限Hより大きく設定する。調整例を以下に示す。

```bash
ros2 param set /ball_detector hsv_lower '[174, 70, 60]'
ros2 param set /ball_detector hsv_upper '[12, 255, 255]'
ros2 topic echo /ball/detection --qos-reliability best_effort
ros2 topic echo /ball/estimate --qos-reliability best_effort --field surface_distance
```

カメラでは輪郭面積・中心・直径・撮影時刻と、レンズ歪みを補正した光学フレームの方向ベクトルを記録する。LD03ドライバは一周の終端に時刻を付けるため、融合ではスキャン中間時刻に最も近い画像を選ぶ。他のドライバが開始時刻を付ける場合は `scan_stamp_is_end: false` を使う。TFで実測オフセットを反映し、点の連続性、ボール断面の円への適合、カメラ上の方向と見かけの大きさで照合する。開始時にカメラとLiDARを照合して対象を確定する。走行中はLiDARだけで直前の位置から連続した円弧を追う。候補なし、複数候補、またはスキャン途絶で停止し、△で再照合する。表面距離はレーザー原点から推定円の近い表面までの距離。横方向は機体中心から推定球中心までのY座標。幾何推定は水平に近い単一の走査面を前提とし、一周内の機体運動の補正は未実装。

## 350mm付近の簡易較正

`config/empirical_350.yaml` は、外付けC270とLD03で直径約60mmのボールを正面・左・右・手前に置いて作成した設定。取り付けや画像サイズを変えた場合は、静止状態でこの対応を取り直す。カメラ画像の中心座標と見かけの直径を、LiDAR点群から推定した方向・距離と照合する。較正した範囲はボール表面距離約0.33〜0.46m、機体正面付近の左右約5°。このモードの自動速度は合成速度で0.035m/sまで。△を押すと `CAMERA_ALIGN` でカメラから横ずれを推定し、横方向のみ最大0.030m/sで合わせる。画像の横中心ではなく、距離に応じた機体正面の画素を基準にする。横移動を止めてから `DETECTING` でカメラとLiDARを照合する。障害物判定が有効な設定では、現在のスキャンで車体周辺の安全を判定できる場合に横移動する。

簡易較正の車体座標 `ball_test_base` はLiDAR中心を原点とし、正面に置いたボールでX前方を定義する。現在のセンサー取り付けの静止プレビューには、`ball_test_base` から `laser` にヨー `-3.13954` radの静的TFを使用する。障害物チェックに使う `half_length` と `half_width` は、LiDAR中心から前後端・左右端への各最大距離を実測して設定する。設定の初期値では自動走行を無効にしている。

### 簡易較正のプレビューを再現する

リポジトリのルートから、ROS 2 とこのワークスペースの setup を読み込んだ端末で起動する。既存の `/scan` がない場合は、LD03の実際のシリアルデバイスを指定して下のlaunchに `lidar:=true lidar_port:=<デバイスパス>` を追加する。

```bash
ros2 launch ball_auto_approach ball_camera.launch.py
```

別端末で次を起動する。

```bash
ros2 run tf2_ros static_transform_publisher --yaw -3.139535756772215 \
  --frame-id ball_test_base --child-frame-id laser
```

さらに別端末で次を起動する。

```bash
ros2 launch ball_auto_approach ball_auto_approach.launch.py \
  camera:=false detector:=false project_root:="$PWD" \
  config:="$PWD/real/src/ball_auto_approach/config/empirical_350.yaml"
```

`/ball/estimate` の `valid` と `surface_distance` で静止状態の照合を確認する。この起動方法では速度は `/cmd_vel_preview` に配信される。既に同じ `laser` の親TFを配信している場合は、静的TFを切り替えてから起動する。

### 2026-11-26 実機試験

- 外付けC270の640×480映像で赤いボールを検出した。LD03のスキャンは約13.5Hz。PS4のL1とスティックによる手動走行を実機で確認した。
- 直径約60mmのボールを用いてカメラとLiDARを簡易較正した。走行開始時のみ照合し、接近中はLiDARで同じ反射点の円弧を追跡する。
- 左右にずれたボールに対し、前進指令ゼロのまま横合わせし、停止後に照合した。LiDARで約397mmから約358mmまで接近し、350mmの許容範囲（340〜360mm）で停止して手動操縦に戻った。
- 実機試験では監視者が走行範囲を確認し、試験時の設定だけで障害物判定を無効にした。リポジトリの既定設定では障害物判定を有効にし、自動走行を無効にしている。別の取り付けやボールでは検証が必要。
- ウォッチドッグのUDP受信を修正し、ハートビート受信と正常終了をCAN非接続で確認した。実機でも監視開始と、試験終了時に4輪の無効化を確認した。

## 段階的な試験

1. 駆動電源を切った状態で、実際のボールを手で450、375、350、340mmに置き、各距離で連続した `/scan` と `/ball/estimate` を記録する。既存ドライバの `range_min=0.05` m はソフトウェアの下限設定。メーカーの[LD03対応共通ドライバ](https://github.com/ldrobotSensorTeam/ldlidar_ros/blob/master/src/main.cpp)にも下限設定がある。350mmでの精度は対象球を使った距離試験で確認する。試験したボールの色・角度・照明条件で10mmの許容幅を満たすか、欠測や異常値も含めて確認する。350mm付近が安定しなければその距離での自動接近は有効にせず、安定して測れる範囲まで `target_distance` を引き上げてから再検証する。
2. `config/params.yaml` のコピーに、上記条件で確認した最小測距を `validated_min_range` として設定し、`auto_enabled: true` を設定する。最小測距は停止許容幅の下端までを評価する。 `config:=/絶対パス/params.yaml` を指定してプレビューで再起動する。以下を別端末で監視し、映像の遮断、スキャン停止、障害物、○、スティック、L1解除、PS4切断で停止を確認する。

```bash
ros2 topic echo /ball/state
ros2 topic echo /cmd_vel_preview
```

3. `/joy` を表示して、SDLの△=3、○=1、L1=9、左縦=1・左横=0・右横=2を実機コントローラーで照合する。足回りのL1インターロックは既存のSDLマッピングを使用する。電源投入前に、`/cmd_vel` の他のパブリッシャーと既存の単独CANドライバを終了する。
4. 操作者が非常停止を確認し、車輪を浮かせて `powered:=true` を追加する。既存CANドライバを `--twist` で起動し、最終指令は `/cmd_vel` に出る。CANインターフェース設定は既存手順に従う。まず車輪の方向、ゼロ指令、0.15秒の指令途絶での無効化を確認する。次に低速の床上試験を行い、停止距離を調整する。`max_vx`、`max_vy` は初回の実機試験では既定値より低く設定できる。
5. スティック中立でL1を離して押すと手動走行を許可する。L1を保持し、中立で△を一度押すと自動接近。○、スティック操作、L1解除で中止する。到達・中止の周期はゼロを出力し、次の周期で手動に復帰する。障害物チェックは走査面に写る点を対象とする。センサーの死角とスキャン面外の物体は操作者が確認する。

`/cmd_vel_manual` と `/cmd_vel_auto` で各モードの生成値を確認できる。切り替えと状態遷移は同じノードで行い、最終トピックの出力元は1つ。`/ball/state` とログに MANUAL、CAMERA_ALIGN、DETECTING、ALIGNING、APPROACHING、COMPLETE、ABORT が出る。終了時にゼロ指令を送信し、Muxが応答しなくなった場合にも足回り側が指令をタイムアウトする。CANドライバの突然の終了には独立watchdogが停止フレームを送る。停止後のアーム処理はCOMPLETE遷移から追加できる。
