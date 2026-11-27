# 実機の引き継ぎ

## 実機ファイル

| 場所 | 内容 |
| --- | --- |
| `real/src/dm_imu/` | 達妙 IMU の ROS 2 ドライバー |
| `real/lidar/ld03_node.py` | LD03 から `/scan` を配信するノード |
| `real/manual_drive/` | コントローラー操作、SocketCAN、実機テスト、既存の作業記録 |
| `real/controller/ps4.yaml` | 既存の PS4 コントローラー設定 |
| `real/tools/` | 車輪と IMU によるオドメトリ、LiDAR フィルター、校正記録 |
| `real/config/` | SLAM Toolbox と RViz の設定 |
| `real/maps/` | 2026-11-24 の地図、保存済みポーズグラフと付随データ |

## 地図作成と操作

- IMU は `ros2 launch dm_imu run_without_rviz.launch.py` で起動できる。デバイスが異なる場合は `ros2 run dm_imu dm_imu_node --ros-args -p port:=<IMUのデバイス> -p imu.calibrate_on_startup:=false` を使う。地図作成時の IMU 入力は `/imu/data`。
- LD03 は `python3 real/lidar/ld03_node.py --ros-args -p port:=<LiDARのデバイス>` で起動する。シェルに入力する際は山括弧を含めて実際のデバイス名に置き換える。
- `real/tools/manual_drive_odometry.py` は車輪の CAN フィードバックと IMU から `/odom` および `odom → base_link` を配信する。既定の CAN は `can0`。駆動コードの既定の参照先は同梱の `real/manual_drive/`。
- 既存セッションの LiDAR の静的 TF は `base_link → laser` が x=0.190 m、yaw=π。フィルター表示に使う `base_link → laser_forward` は x=0.190 m、yaw=0。再設置時は取付位置と向きに合わせる。
- フィルター経由では `real/tools/filter_ld03_scan.py` が `/scan_filtered` を出力する。SLAM Toolbox の `online_async_launch.py` に渡す `slam_params_file` は、フィルターありなら `real/config/manual_mapping_filtered.yaml`、元のスキャンなら `real/config/manual_mapping.yaml` の絶対パスを指定する。RViz は `real/config/manual_mapping.rviz` を使用する。
- コントローラーでの実機操作と停止手順は [手動走行ワークスペースの README](../real/manual_drive/README.md) を参照する。既存セッションの Joy ノードは `harurobo_teleop` の `stage_a.yaml` を使用していた。

## 2026-11-25 フォルダ整理

既存の実機ファイルを `real/` に集約した。この PC で稼働中だったセッションのため、旧ルートの `build/`、`install/`、`log/` と、`src`、`config`、`maps`、`tools`、`sh` の互換リンクをローカルに残した。別 PC での作業には `real/` 配下を使う。ホーム直下の手動走行・LiDAR・コントローラーの元フォルダも残している。

作業者が確認していた機能は IMU と LiDAR による地図作成とコントローラー操作。移設後の別 PC での動作確認、シリアルデバイス名とコントローラー機種の記録、シミュレーション実装が当時の残作業。

移設後、この PC の Ubuntu 24.04 / Jazzy で IMU 側 2 パッケージと手動走行側 4 パッケージのビルドが完了した。既存の実機セッションは稼働を続けており、新しい配置からの実機起動は未確認。

## シミュレーション初版

Gazebo Harmonic の簡易機体と練習用ワールド、既存のコントローラー入力、SLAM Toolbox を接続した。起動、操作、地図保存は [シミュレーションの手順](../simulation/README.md) を参照。仮の機体寸法とセンサー位置を使っている。実機の寸法測定と車輪・駆動系のモデル化は今後の作業。

この PC で前進・後退・左右移動・旋回、指令途絶後の停止をオドメトリで確認した。模擬 Joy 入力で既存 teleop 経由の走行と L1 解放時の停止を確認した。仮想 LiDAR と IMU、シミュレーション時刻、SLAM の地図配信とポーズグラフ保存も確認した。物理コントローラーでの操作と別 PC での再現は未確認。

シミュレーションの既定フィールドを関東春ロボコン2027に変更した。図面の版、仮想 LiDAR の高さ、固定の競技物モデル、以前の練習用フィールドの選び方は [シミュレーションの手順](../simulation/README.md) に記録した。

## 2026-11-27 GUI・カメラ・LiDAR較正

[当日の開発記録](progress-2026-11-27.md)に実装、確認結果、起動手順、残作業を記録した。外部較正システムは実装済みだが、確認時点のextrinsic_2.yamlは6点のサンプル保存までで、実機の外部較正は未完了。次は広い分布のサンプルを追加して推定・独立検証・再保存を行う。
