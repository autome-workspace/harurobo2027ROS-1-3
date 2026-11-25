# 春ロボ ROS 2 手動走行

Ubuntu 24.04 / ROS 2 Jazzy。段階 A（コントローラー入力）と段階 B の受信診断を含む。駆動ノード・CAN 送信は含まない。実機確認はモーター未接続で行う。

```bash
cd real/manual_drive  # プロジェクトのルートから
source /opt/ros/jazzy/setup.bash
colcon build --packages-select harurobo_teleop
colcon test --packages-select harurobo_teleop
colcon test-result --verbose
source install/setup.bash
export ROS_DOMAIN_ID=53 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch harurobo_teleop stage_a.launch.py
```

操作は左スティック＝前後・左右、右スティック横＝旋回。起動後や入力途絶後は、両スティックを中立にして L1 を離し、中立のまま L1 を押し直す。L1 を離すと停止指令。`/manual/armed` は表示・試験用で、独立したトピックのため駆動側の許可には使用しない。後続段階では指令と許可を一体で扱う受信監視を追加する。加速度制限は後続の機体側に実装し、異常停止はその制限を経由しない。

[段階 A の実機受入手順](docs/stage-a-check.md)

## 段階 B：SocketCAN 受信診断

`harurobo_base/can_monitor` は読み取り専用で、上記の段階 A の launch からは起動しない。駆動指令、速度換算、停止フレームはまだ未実装。

```bash
colcon build --packages-select harurobo_base
source install/setup.bash
ros2 run harurobo_base can_monitor --ros-args \
  --params-file src/harurobo_base/config/stage_b_monitor.yaml
```

別端末で `ros2 topic echo /diagnostics`。詳細は [段階 B の準備・検証手順](docs/stage-b-socketcan.md)。

## 機器確認待ちで先行実装した制御コア

4輪逆運動学・全輪同率飽和・ヨー保持を非駆動の C++ コアとして実装。[範囲と残件](docs/control-core.md)を参照。車輪配置は未計測、IMU/CAN 駆動統合は未完了。

## 2026-11-20 の全体記録

成果・既知の問題・ハマった点のまとめは [lessons-2026-11-20.md](docs/lessons-2026-11-20.md)。

## 継続手動走行の起動

```bash
./scripts/start_manual_drive.sh   # 起動（can0確認・二重起動防止つき）
./scripts/stop_manual_drive.sh    # 安全停止
```

## 段階 D：IMU受信（非駆動）

`harurobo_imu` の受信専用ドライバを追加。起動時の設定保存や自動校正は行わない。[起動・検証手順](docs/stage-d-imu.md)を参照。実機の受信・軸確認とヨー保持の駆動統合は未完了。

非駆動の `harurobo_bringup preview.launch.py` でDS4・IMU・ヨー保持プレビューを一括起動できる。駆動電源OFFで使い、出力トピックをモーターに接続しない。詳細は段階D手順を参照。
