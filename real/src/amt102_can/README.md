# AMT102-CANエンコーダー受信

2026-11-28版AMT102-CANプロトコルに従い、標準CAN ID 0x360〜0x367・DLC 6を受信します。モーター制御や設定変更、SYNC送信は行いません。Python標準のSocketCANを使用し、追加のpython-canは必要ありません。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths real/src --packages-select amt102_can --symlink-install
source install/setup.bash
ros2 run amt102_can encoder_node --ros-args --params-file real/src/amt102_can/config/encoders.yaml
```

Control Centerの「AMT102 エンコーダー（2台）」からも起動できます。CANがDOWNでも待機し、GUIの「エンコーダーCANを有効化」後に再接続します。自動でCAN設定を変更する処理はありません。既存CAN受信ノードと同時に使用でき、他の受信ノードのフレームを消費しません。

## 表示とトピック

- `/encoders/state`（std_msgs/String、JSON、既定50 Hz）: ch、受信状態、角度、相対回転数、規格位置uint24、受信Hz、故障bit、同期状態、最終受信からの時間。
- `/encoders/chN/joint_state`（sensor_msgs/JointState、N=0〜7）: 受信開始時からの相対角度rad。位置を受信した時刻をヘッダーに保持し、新しいデータのないときは配信しません。

位置は1回転65536の規格値で、AMT102Vの生カウントではありません。1回転内の角度は0〜360°、相対回転数はこの受信ノードが最初に受信した位置を0とします。24 bitの正負折り返しを処理します。隣接受信の移動量が128回転未満という仕様上の前提があります。0.5秒以上受信が途絶えた後は相対位置の基準を再設定します。チャンネル不明の起動通知0x018を受けたときは全chの基準を再設定します。通知を受けられなかった再起動やCPR・反転の変更を確実に検出することはできないため、その場合はノードも再起動してください。

角度は基板のCPR設定が実センサーのPPR×4と一致する前提です。未確認のCPRをホスト側で8192と仮定して生カウントへ換算しません。15 bit時刻の折り返しやSYNC切り替えがあるため、この初版では速度を算出しません。

## 接続の注意

2台は異なるchに設定してください。起動通知だけでは台数を識別できません。同一chの2台を別々に表示することはできず、同一ch衝突をこの受信だけで確実に判別することもできません。既定は両方ch 0なので、基板のSW1で片方をch 1などへ変更します。元資料に従い、軸を止めて1.5秒長押し→設定ch回の短押し→1.5秒長押しで保存・再起動します。

通信はClassical CAN・1 Mbpsです。CAN_H/CAN_L、GND、電源、バス両端の終端を確認してください。起動直後は再起動履歴のためfault bitが立つ仕様です。異常詳細の設定5を読み出し、再起動履歴だけなら「再起動履歴のみ（受信正常）」と表示します。その他の履歴や未取得の詳細は警告表示します。履歴クリア、CPR・方向変更は送信しません。SYNCなしで位置を受信できるため、未同期でも受信異常にしません。

センサー未接続と軸停止はプロトコル上区別できません。チャンネル受信は基板との通信確認であり、軸の健全性の保証ではありません。

```bash
python3 -m pytest real/src/amt102_can/test -q
ros2 topic echo /encoders/state
ros2 topic echo /encoders/ch1/joint_state
ros2 topic echo /encoders/ch2/joint_state
```

## USB-CANが2台ある場合

このPCはDM3519用のgs_usbがcan0、AMT102用のWeAct USB2CANV2が仮想シリアルから作るslcan0です。エンコーダーノードの既定接続先をslcan0にしています。共有設定でモーター用can0へ暗黙に接続しません。別構成のPCではinterfaceパラメータとControl CenterのENCODER_CAN_INTERFACEを明示的に変更してください。

WeActの機器は/dev/serial/by-id/のUSBシリアル番号で識別します。このPCでは末尾B245206F33A6の機器で、ttyACM番号が変わっても同じものを指定します。Control CenterのPC固有設定にはENCODER_CAN_INTERFACEとENCODER_CAN_PORTを分離して保存します。

[WeAct公式仕様](https://github.com/WeActStudio/WeActStudio.USB2CANFDV2)に基づき、GUIのエンコーダーCAN有効化で既存のslcandを使ってSLCANを1 Mbpsで開き、slcan0をUPにします。初回は管理者認証が必要です。既に作成済みならUP操作のみ行い、slcandを重複起動しません。この操作はエンコーダー側だけを対象とし、モーター側can0の設定を変更しません。slcandはネットワーク接続のためのデーモンで、GUI終了後も接続を保持します。USB再接続時は再び有効化してください。

手動で設定する場合の例です。実際の固定デバイス名に置き換えます。

```bash
sudo slcand -o -c -s8 /dev/serial/by-id/<WeActの固定名> slcan0
sudo ip link set dev slcan0 up
```

ノードから送るCANフレームは設定5の読み出し要求（0x480、操作0）だけです。GUIによるUSB-CANのオープン・ビットレート設定と、エンコーダー基板への設定書き込みは別の操作です。

2026-11-28、このPCのslcan0でCH1・CH2の位置データと設定読出し応答を確認しました。両方の設定送信周波数は200 Hz、異常詳細は再起動履歴0x00000001のみでした。表示ノードから設定書き込みや履歴クリアは行っていません。

## 異常履歴の表示

位置フレームのfaultは履歴の有無だけを示し、故障箇所の種類は含みません。faultが立つ受信中chだけに、設定5の読み出しを各ch約1秒間隔で送ります。要求は同時に1件とし、ch・操作・項目・seq・200 msの応答期限を照合します。起動通知・CAN切断・fault bitの変化で詳細キャッシュを無効化します。取得後1.5秒以上経った詳細は正常判定に使いません。

詳細が0x00000001だけなら情報表示・緑、ほかのbitを含むときは内容と16進値を警告表示します。読み出し未応答・エラー・古い詳細も警告のままです。rawのfault値は変更せず、/encoders/stateにfault_detail、fault_detail_age、fault_level、fault_textを追加します。bitが常に1のままでも詳細を定期更新し、追加の異常履歴を取りこぼさないようにします。表示は直近の履歴読み出し結果で、履歴を消去する処理はありません。

2026-11-28に実機のCH1・CH2で詳細0x00000001を読み出し、再起動履歴のみの分類を確認しました。

## Control Centerへの追従表示

ROSの状態・位置配信を50 Hzとし、GUI用監視プロセスはエンコーダーの受信ごとに軽量通知を転送します。ROSグラフ探索の2 Hz更新は分離し、GUIは通常の重い画面更新を待たずに角度ラベルを更新します。監視JSONは全8chでも切れないよう64 KiBまで、通常ログは従来の4 KiBに制限します。表示は角度4桁・相対回転数6桁です。

CANの有効化と読取ノードの起動は別です。ダッシュボードの「エンコーダー読取開始」で値の読取を開始し、停止中は停止状態を明記します。「エンコーダーCANを有効化」から有効化した場合は、CANがUPになりROS監視が準備でき次第、読取も開始します。受信途絶は最新の受信時刻で判定し、以前の値を新しい時刻で更新しません。

このPCの以前のscripts/amt102v_can.pyと同じbig-endian uint24・1回転65536の換算を使用します。2026-11-28に手で回したCH2の生データとROS値の変化を確認し、模擬8chによるROS→監視→Qtの表示テストで1.2秒に60種類の値の更新、受信停止と復帰を確認しました。実機のモーターは起動していません。

## RVizでCH1・CH2の回転を見る

Control Centerの実機モードで「RViz2（エンコーダー）」を起動します。エンコーダー読み取りも依存順に起動し、各chの矢印と角度・相対回転数を表示します。専用ウィンドウを閉じると表示ノードも終了します。元の読み取りノードは独立して使えます。

既に読み取りノードが動いていれば、端末から次のコマンドでも開けます。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=53 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch amt102_can view.launch.py
```

手動でRVizに追加する場合は、表示ノードencoder_visualizerを起動し、Fixed Frameをamt102_view、Add→By topic→/encoders/markers→MarkerArrayとします。この表示は角度の確認用の模式図です。chの並べ方や円の大きさは実機の取り付け寸法を表しません。表示専用のamt102_visualization→amt102_viewだけを配信し、ロボットのTFには接続しません。

位置の実受信時刻から0.5秒以上古い値、受信途絶、切断は表示しません。マーカーの寿命も0.6秒に制限し、読取が止まったときの古い矢印を残しません。
