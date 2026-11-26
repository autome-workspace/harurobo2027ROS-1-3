# ROS 2 構築プロジェクト

作業環境は Ubuntu 24.04 と ROS 2 Jazzy。

| フォルダ | 内容 |
| --- | --- |
| [real/](real/) | 実機の IMU、LiDAR、地図作成、コントローラー操作、[ボール自動接近](real/src/ball_auto_approach/README.md) |
| [simulation/](simulation/) | シミュレーション用。現在は未着手 |
| [docs/](docs/) | [PC セットアップ](docs/setup.md)と[実機の引き継ぎ](docs/handover.md) |

実機側の変更は `real/`、シミュレーション側の変更は `simulation/` に置く。設定、スクリプト、引き継ぐ地図は Git で管理する。各ワークスペースの `build/`、`install/`、`log/` は各 PC で生成する。

作業時は、変更した設定、確認に使った機材と手順、結果、残作業を [引き継ぎ記録](docs/handover.md)に追記する。
