# ROS 2 Control Center

Ubuntu 24.04 / ROS 2 Jazzy向けの日本語ローカル管理GUIです。PySide6の画面、プロセス管理、ROS状態監視を分離しています。

## 起動と登録

リポジトリのルートから実行します。

```bash
bash ros2_control_center/install.sh
```

Ubuntuのアプリ一覧から「ROS 2 Control Center」を起動できます。デスクトップにも登録する場合は `--desktop` を付けます。インストーラーはユーザー用の仮想環境を作成し、システムPythonを上書きしません。ROS関連パッケージは各ワークスペースで事前にビルドしてください。

開発時は次の方法でも起動できます。

```bash
"$HOME/.local/share/ros2-control-center/venv/bin/python" ros2_control_center/main.py
```

## 利用できる機能

- 実機・Gazebo・デモの切り替え、個別起動停止、依存順の一括起動停止。
- 二重起動防止、管理外ノード検出、終了コードとトピック受信を使った状態表示。
- 管理中のプロセス群をSIGINT、SIGTERM、SIGKILLの順で停止。
- プログラム別・統合ログ、検索、保存、保持件数の制限。
- USB・シリアル機器の表示とSocketCAN診断、認証画面を経由したCANのUP操作。
- PS4手動操作と既存のボール自動接近処理の起動。
- カメラ撮影設定、内部較正、[カメラ・LiDAR外部較正](../real/src/camera_lidar_calibration/README.md)と較正後の画像投影。

共有の起動コマンド・依存関係は `config/programs.yaml` にあります。PC固有のデバイス、撮影設定、較正ファイルなどは `$XDG_CONFIG_HOME/ros2-control-center/local.yaml`（未指定時は `~/.config/ros2-control-center/local.yaml`）に保存します。例は `config/local.example.yaml` を参照してください。管理・監視機能の全項目が実装済みという意味ではありません。任意ノードのパラメータ編集などは今後の拡張です。

## カメラと較正

「設定」→「カメラ撮影設定」で対応解像度と内部較正YAMLを選び、保存後にカメラを再起動します。内部較正は9×6交点・25 mmマスのチェッカーボードを使用します。解像度ごとに較正を保存し、サイズが異なる較正は適用しません。

このPCでは1280×720で横方向の撮影範囲が広がることを確認しました。機種ごとの画角は実機で確認してください。外部較正GUIから撮影設定とHSV調整も行えます。色設定を既存のボール認識に反映するには認識ノードを再起動します。

外部較正・投影パッケージは次のようにビルドします。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths real/src --packages-select ball_auto_approach camera_lidar_calibration --symlink-install
```

## 安全性

実機駆動は明示操作と確認を必要とします。較正と実機駆動の同時起動を防止し、較正プログラムから移動指令は送信しません。停止・指令途絶への対処には既存の駆動側監視も使用します。GUIの停止ボタンは物理非常停止の代替にはなりません。管理外のプロセスは勝手に停止しません。

## ハードウェアを使わない確認

```bash
source /opt/ros/jazzy/setup.bash
QT_QPA_PLATFORM=offscreen "$HOME/.local/share/ros2-control-center/venv/bin/python" ros2_control_center/tests/smoke.py
QT_QPA_PLATFORM=offscreen "$HOME/.local/share/ros2-control-center/venv/bin/python" ros2_control_center/tests/camera_settings.py
```

デモは実機と別のROSドメインを使用します。外部較正の合成データ・ROS経路・GUIテストは[パッケージのREADME](../real/src/camera_lidar_calibration/README.md)を参照してください。実機の外部較正完了と独立位置での精度確認は今後の作業です。
