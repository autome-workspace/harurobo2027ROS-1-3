# Ubuntu 24.04 でのセットアップ

別 PC では ROS 2 Jazzy、colcon、rosdep、Python の pyserial、Joy、SLAM Toolbox を用意する。rosdep の初期化が済んでいることを確認する。

プロジェクトを取得したら、リポジトリのルートから以下を実行する。serial は Git サブモジュールで、親リポジトリに記録されたリビジョンを取得する。

```bash
git submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
(
  cd real
  rosdep install --from-paths src manual_drive/src --ignore-src -r -y
  colcon build --base-paths src
)
(
  cd real/manual_drive
  colcon build --base-paths src
)
```

実機ツールのシェルでは `source real/install/setup.bash`、コントローラー操作のシェルでは `source real/manual_drive/install/setup.bash` を実行する。いずれもリポジトリのルートからのパス。

PC を替えたら IMU と LiDAR のシリアルデバイス名、アクセス権、CAN インターフェース、コントローラーの認識を確認する。IMU と LiDAR のデバイス名は各ノードの `port` パラメーターで設定する。IMU の既存 launch ファイルは `/dev/ttyACM0` を指定する。別名のデバイスを使う場合は、引き継ぎ手順にある直接起動のコマンドで指定する。起動する端末間で `ROS_DOMAIN_ID` をそろえる。

元の IMU ドライバーの説明は [実機側 README](../real/README.md) に残している。そこにある Ubuntu 22.04 / Humble は元のドライバーの検証環境で、このプロジェクトの作業環境は Ubuntu 24.04 / Jazzy。
