# 段階 D：DM-IMU-L1 非駆動での取り込み

## 実装と既存コード

`harurobo_imu/imu_receiver` は 921600 baud / 8N1 の受信専用ドライバ。`port` は明示指定を必須とし、設定コマンド・自動校正・フラッシュ保存・CAN送信は行わない。ドライバはTFを発行しない。装着位置と軸を確認してからbringup側で静的TFを用意する。

既存の `/home/kaboc/DM_IMU_ros2` はそのまま保存（Git HEAD `68597cf6bea7d99808c23ccf203b555c25527acd`、開始時点でCRCヘッダーの変更と未追跡ファイルあり）。そのドライバは起動時に1000 Hzと保存を設定し、固定57バイト読みの長さを検証せずCRC失敗時の前回値を混ぜ得るため、走行系に直結しない。

フレーム仕様の照合資料：CoRE2026 `core_damiao_imu` の `protocol.py`、コミット `050d37f52cf0b03fdee6f59fceeca5a8ca99ca95` と既存DMドライバ。19バイト `55 AA slave RID [float32 LE x3] CRC16 LE 0A`、CRCはCCITT（poly 0x1021/init 0xFFFF）。加速度はm/s²、角速度はrad/s、Eulerはdegreeという仕様を採用し、実機でも検証する。slave IDは初期値1のみに対応。

- CRC・末尾・RID・有限値を確認し、ストリームを再同期。
- 新しいRID 1→2→3の一式が20 ms以内に揃った場合のみ `/imu/data` を発行。
- シリアル切断・CRC失敗・バックログ超過では受信キャッシュを破棄。切断後は1秒周期で再接続。100 ms完全サンプルなしで `/diagnostics` をSTALE。
- PC受信時刻をstampに使用。デバイス内の取得時刻・サンプル同期は保証しない。QuaternionはZYXから正規化。測定に基づく共分散は未設定（ROSの全ゼロ=未知）。

## 検証済み（実機ではない）

Jazzyでビルド、GoogleTest 7件に合格。独立ROS domain 56のPTY結合テストで、断片化、SI値・Quaternion、CRC破損時の無発行、データ途絶での発行停止、切断・再接続時の前回値破棄を確認。

```bash
cd ~/ros2-poc-manual-drive
source /opt/ros/jazzy/setup.bash
colcon build --packages-select harurobo_imu
colcon test --packages-select harurobo_imu --return-code-on-test-failure
source install/setup.bash
ROS_DOMAIN_ID=56 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 src/harurobo_imu/test/check_imu_pty.py
```

## 実機受入（未実施）

駆動電源OFF、モーターベンチ終了を確認してから、IMUをデータ対応USBで接続する。`ls -l /dev/serial/by-id/` から対象のパスを選ぶ。純正USB-CANのttyとは取り違えない。旧IMUドライバを同時に起動しない。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2-poc-manual-drive/install/setup.bash
export ROS_DOMAIN_ID=53 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch harurobo_imu receive.launch.py port:=/dev/serial/by-id/実機のパス
```

最初はIMUだけ静止させて、加速度の大きさが約9.81 m/s²、gyroが約0、Quaternionノルム約1、頻度とCRCエラーを確認する。その後、手で反時計回りに回しyawとgyro_zの正符号を確認。データ欠落、USB抜去/再接続で古い値を出し続けないことを検証する。200 Hzは目標開始値であり、この受信専用ドライバは出力レートを変更しない。設定が合わず一式が揃わない場合は、その内容を確認してから別途明示的な揮発設定を検討する。

ヨー保持コアは `harurobo_base/heading_hold.hpp` のみ実装済み。IMU検証・機体軸TF・実機ゲイン調整・駆動への統合は残件。今回IMUを使った駆動は行わない。

## 2026-11-20 実機受信の結果

- USB `6877:4d55` DM-Tech DM-IMU-L1、固定パス `/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00`。USBハブ/ケーブルの変更後に列挙。
- 設定を書かない初回受信で約989 Hz。実機は RID 1/2/3 の19バイトに加えてRID 4の23バイトQuaternionフレームも出力していた。当初この別形式をinvalidとして数えたため、RID4はCRCを検証したうえで読み飛ばすよう修正した。IMUへの設定変更はなし。テストを1件追加し、単体8件とPTY結合試験が合格。
- 修正後の5秒計測：4961サンプル、約991.88 Hz、最大受信間隔約2.96 ms。加速度ノルム平均9.8016 m/s²、gyro_z平均 -0.0000685 rad/s、Quaternionノルム約1。yaw変化 -0.00728°、範囲0.01989°（短時間の観測値で、長期ドリフト仕様や校正合格を示すものではない）。診断invalid_frames=0、backlog_drops=0。ドライバ終了済み。
- 生データ `log/stage-d-imu-static.jsonl`、集計 `log/stage-d-imu-static-summary.log`。受信時刻とデバイス取得時刻の差は未計測。目標200 Hzへの設定変更は未実施。実機のyaw/gyro_z方向、軸と装着方向、USB抜去復帰、長期静止ドリフトは今後確認する。

- 操作者が上から見て反時計回りに回した再試験（120秒）：開始/終了2秒平均yaw差 81.477°、gyro_z積分 81.079°、絶対値0.1 rad/s以上の正/負サンプル 2293/252。ログ `log/stage-d-imu-direction-retry.jsonl`、集計 `log/stage-d-imu-direction-retry-summary.log`。

## ヨー保持の非駆動プレビュー

`harurobo_base/heading_preview` は `/imu/data` と `/cmd_vel/manual` を受け、`/cmd_vel/manual/heading_preview` に補正後の速度を発行する。CANを開かず、許可信号も出さない。`base_link` と `imu_link` の軸が同方向の前提であり、装着方向が違う場合はまだ使わない。静的TFは未計測の位置・姿勢を仮定して発行しない。

- Kp=0.5、Kd=0、補正上限0.2 rad/s。旋回入力中は手動優先。平行移動中、gyro_zが十分落ち着いて200ms経過したところで目標yawを捕捉。中立で目標解除。
- 受信時刻を単調時計で監視（IMU100ms・指令150ms）し、メッセージstampも検証。Quaternionノルム・有限値・frame_id・時刻逆行/重複を確認する。
- 異常後はゼロ出力とし、新鮮なIMU/指令と中立が揃うまで補正を再開しない。これはプレビューの復帰であり、実機の駆動再許可ではない。
- ROS domain57 の模擬入力結合テストに合格（角度捕捉、補正方向・上限、手動旋回優先、中立解除、IMU途絶保持、正常復帰だけでは再開なし、不正Quaternion・古いstamp拒否）。ログ `log/stage-d-heading-preview-test.log`。

駆動電源OFF、IMUの+Xを機体前、+Zを上に剛体固定後、DS4/IMU/プレビューだけを一括起動：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2-poc-manual-drive/install/setup.bash
export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch harurobo_bringup preview.launch.py \
  imu_port:=/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00
```

別端末でもdomain58を指定し `ros2 topic echo /cmd_vel/manual/heading_preview` を確認する。L1と平行移動入力を保持し、IMUを反時計回りに回すと捕捉済み目標への補正angular.zは負。手動旋回が優先され、L1解除・中立ではゼロ。IMUは動かして検証できるが、この段階で車輪は動かさない。実機でのプレビュー試験とゲイン調整・CANへの統合は未実施。

## 2026-11-20 ヨー保持プレビュー実機確認（非駆動）

- 初回と再試行の記録は**無効**：段階A試験由来の旧launchを含め複数組のノードが重複起動しており、修正前テレオプの中立ノイズも混入していた。全プロセス停止と単一構成確認後にやり直した。以降は計測前に各トピックの発行者数を確認する。
- あわせて `/joy` 右スティック中立ノイズ対策として旋回軸専用のデッドゾーン（0.12）を追加し、単体テスト1件を追加（合計44件合格）。
- クリーンな3回目（DS4・IMU・プレビュー各1組、domain 58、120秒、駆動電源OFF・CAN送信なし）：平行移動保持中に機体を反時計回りに回したところ、保持中のサンプル1560件・回転中246件で、補正angular.zは **負方向で-0.2 rad/sの上限に達した**。回転量は約68.4°、保持中の入力異常・手動旋回入力は0件。yaw・gyro_zは反時計回り正で合致。生ログ `log/stage-d-preview-real-retry3.jsonl`。
- 実機での未確認：装着軸の実測（+X前方/+Z上の前提は未検証）、長期静止ドリフト、USB抜去復帰後の古いデータ破棄、200 Hz化、ゲイン調整（Kp=0.5・Kd=0は未調整）。プレビュー出力をCAN駆動に接続する統合は未実施であり、この検証は駆動受入ではない。

## 2026-11-20 ヨー保持とCAN駆動の統合（浮上ベンチ）

### 途中で発生した停止と原因

- 許可直前の静止確認で collect が100 ms無応答で中断を2回以上。 プロセス内診断（単送信・間隔ありバースト）は同一ソケットで全台0.5 ms応答と正常。
- 確定原因：CANable の USB再列挙後、フレーム間隔を空けない連続送信で3本目以降がサイレントに欠落（バスエラー計数0）。1 ms以上の間隔で全台応答。`bench_ds4_four.speeds()` への送信間隔パッチが初回は文字列不一致で未適用だったことが、ベンチ側の失敗の直接原因。適用を `grep` で検証し、単発・有効化・終了後処理の各送信ループにも同じ間隔を追加。
- 60秒枠内に操作が間に合わないケースがあり、ベンチ枠を120秒、許可を10秒に拡大（許可は依然として自動失効）。

### 統合結果（駆動電源ON・全輪浮上・CAN送信あり）

- 事前確認（全台無効・設定・初期静止・IMU新鮮）→ L1許可 → 静止確認合格 → 全輪駆動許可 → 並進指令 → gyro収束200msで目標yaw捕捉（holding=True）→ 10秒経過で全輪停止・無効返信確認。試験中のCANエラー・応答途絶・IMU異常なし。
- 保持中に機体を反時計回りに回したところ、全輪共通の旋回成分が **0 → −0.0005 → −0.0125 → −0.0280 rad/s** と負方向へ増加。回転（正）を打ち消す向きの補正であり、Kp=0.5 ならyaw偏差約3.2°相当。上限−0.2 rad/sには未達（回転角が小さかったため）。
- 生ログ `log/stage-d-heading-bench-10s.log`。操作者は回転・停止・異常なしを確認済み。

### 未確認（次の課題）

- 補正上限到達時の全輪反応、大きな回転角での応答、手動旋回優先の車輪上での再確認、IMU途絶時の駆動停止（実機）、ゲイン調整（Kp=0.5/Kd=0 は初期値）。
- 床上走行は引き続き条件付き保留：`TIMEOUT=0` の意味と通信断時の自力停止が未確認。手動非常停止はあるがPCハートビート遮断ではない。

## 読み取り専用ステータスGUI（tkinter + rclpy）

`harurobo_bringup robot_status_gui.py`（`src/harurobo_bringup/gui/robot_status_gui.py`）。tkinter 標準ライブラリのみで、`/diagnostics`、`/imu/data`、`/cmd_vel/manual`、`/manual/armed`、`/cmd_vel/manual/heading_preview` を購読して一画面表示。CAN・シリアルにはアクセスせず、何も発行しない。IMU の受信レート・最終サンプル経過時間（100 ms で赤）、指令の新鮮さ（150 ms で赤）、診断レベルを色分け表示。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2-poc-manual-drive/install/setup.bash
export ROS_DOMAIN_ID=58 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run harurobo_bringup robot_status_gui.py
```

既知の制限：`/manual/armed` は表示用トピックであり駆動許可に使わない設計は不変。IMUレートは受信コールバック数の概算で、GUIの描画負荷の影響を受ける。
