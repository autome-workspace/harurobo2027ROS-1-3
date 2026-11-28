# ROS 2 構築プロジェクト

作業環境は Ubuntu 24.04 と ROS 2 Jazzy。

| フォルダ | 内容 |
| --- | --- |
| [real/](real/) | 実機の IMU、LiDAR、地図作成、コントローラー操作、[ボール自動接近](real/src/ball_auto_approach/README.md) |
| [ros2_control_center/](ros2_control_center/) | 日本語の起動管理GUI、カメラ設定、内部・外部較正への入口 |
| [simulation/](simulation/) | Gazebo での走行、コントローラー操作、地図作成 |
| [docs/](docs/) | [PC セットアップ](docs/setup.md)と[実機の引き継ぎ](docs/handover.md) |

実機側の変更は `real/`、シミュレーション側の変更は `simulation/` に置く。設定、スクリプト、引き継ぐ地図は Git で管理する。各ワークスペースの `build/`、`install/`、`log/` は各 PC で生成する。

作業時は、変更した設定、確認に使った機材と手順、結果、残作業を [引き継ぎ記録](docs/handover.md)に追記する。

最新の作業まとめ: [2026-11-28 開発記録](docs/progress-2026-11-28.md)。AMT102-CAN受信、指定距離PID移動、横ずれ補正、GUI・較正の改善を記録しています。[エンコーダー受信](real/src/amt102_can/README.md)、[Control Center](ros2_control_center/README.md)、[カメラ・LiDAR外部較正](real/src/camera_lidar_calibration/README.md)に詳しい手順があります。
