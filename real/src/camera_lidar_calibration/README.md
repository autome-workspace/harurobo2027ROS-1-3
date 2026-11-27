# カメラ・2D LiDARの外部較正

停止中のロボットで球を前後左右へ移動させ、カメラ画像とLiDARの対応から6自由度の外部変換を求めます。各サンプルのボール位置やセンサーの取り付け寸法を測定する必要はありません。ボールの直径と、球中心から走査面までの高さ差は必要です。

既存のカメラドライバー、LD03ドライバー、内部較正YAMLを利用します。GUIと較正アルゴリズムは独立しています。モーター制御プログラムへの変更や移動指令の送信はありません。

## 環境と調査結果

Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12を使用します。このPCでOpenCV 4.6.0、NumPy 1.26.4、SciPy 1.11.4、rclpy、cv_bridge、tf2_rosを確認しました。

| 入力 | 確認した構成 |
| --- | --- |
| カメラ | 既存のball_auto_approach/usb_camera、WCM USB WEB CAM、調査開始時640×480／現在1280×720、camera_optical_frame |
| 画像 | /camera/image_raw、sensor_msgs/msg/Image、BGR8 |
| 内部較正 | /camera/camera_info、sensor_msgs/msg/CameraInfo。保存済み内部行列と歪み係数を再利用 |
| LiDAR | 既存のreal/lidar/ld03_node.py、/scan、sensor_msgs/msg/LaserScan、laser |
| 時刻 | LD03の一周終端の時刻からscan_time/2を引いて画像時刻と対応付け |
| 認識 | 既存認識と同じHSVラップ、凸包、円形度・充実度の判定を、輪郭・信頼度も返す独立関数に実装 |

既存の認識トピックは輪郭と信頼度を持たないため、このパッケージは画像から認識情報を生成します。既存認識ノードの起動は必要ありません。センサードライバーとその設定ファイルは変更していません。

## ビルドと起動

リポジトリのルートから実行します。

~~~bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths real/src --packages-select camera_lidar_calibration --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=53
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch camera_lidar_calibration calibration.launch.py
~~~

カメラとLiDARを先に起動してください。Control Centerからは実機モードの「カメラ・LiDAR 外部較正」を選択できます。この場合はカメラとLiDARも依存順に起動します。手動駆動・自動接近との同時起動はGUIで防止します。

Qtは既存Control Centerのユーザー用仮想環境を利用します。システムPythonのパッケージは上書きしません。別のPySide6対応Pythonを使う場合はCAMERA_LIDAR_GUI_PYTHONに実行ファイルを指定します。ROSと同じPythonバージョンで、ROSモジュールを参照できる環境を使用してください。

~~~bash
export CAMERA_LIDAR_GUI_PYTHON="$HOME/.local/share/ros2-control-center/venv/bin/python"
~~~

GUIなしでも較正ノードを起動できます。

~~~bash
ros2 run camera_lidar_calibration calibration_node
~~~

## 内部較正

「内部較正」ボタンから既存Control Centerの較正画面を開きます。9×6の内部交点、25 mmのマスに対応します。既に内部較正が済んでいれば再実行は不要です。取り付け位置や角度だけを変えたときは外部較正だけをやり直します。

内部較正画面で新しいYAMLを保存した場合は、カメラを停止・再起動するか、外部較正の設定でそのYAMLを直接指定してください。

Control Centerのソースを別の場所に配置した場合は、CONTROL_CENTER_DIRにmain.pyがあるディレクトリを指定してください。

CameraInfoに有効な行列がない場合は、「設定」の内部較正YAMLを指定できます。YAML指定時はその内容を優先し、CameraInfoは解像度とフレームの一致を確認します。機種・レンズ・解像度を変えた場合は適合する内部較正を選び直してください。

## ボールと高さ

共有設定の初期値は直径60 mm、高さ差0です。初期較正では球中心と走査面の高さを合わせると扱いやすくなります。

高さが異なる場合は「球中心−走査面の高さ」を設定します。このPCでは、台35 mm、直径60 mm、走査面80 mmの申告値から、球中心65 mm、高さ差−0.015 mをPC固有設定に保存しました。

~~~text
断面半径 = sqrt(球の実半径² − 高さ差²)
球中心 = (X, Y, 高さ差)  # LiDAR座標、単位m
~~~

走査面が球に交差しない設定は拒否します。球中心のZは設定した高さ差で決まります。2D LiDARから未知の球高さを推定する処理は含みません。全サンプルで同じ高さを使用してください。高さを変える場合はサンプルを取り直します。

## 外部較正の操作

1. 両センサーで球全体が見える場所に球を静止させます。遮蔽や強い反射を避けます。
2. 点群画面の黄色い円がLiDARの候補です。複数ある場合は、対応する球の円をクリックします。水色が選択対象です。「LiDAR自動選択」は候補が1個のときだけ選択します。
3. 「静止」表示後に「サンプル取得」を押します。「自動取得」は静止を検出して取得します。同じ場所の連続取得は除外します。
4. 球を近距離・遠距離・左・右に移動させ、20〜30か所を取得します。緑が学習用、ピンクが検証用です。既定では5個ごとに検証用となります。「次の手動取得は検証用」でも指定できます。
5. 「外部較正実行」を押します。学習点の分布不足、解の不安定、独立検証の不足は成功にしません。
6. 推定に使っていない位置に球を置き、検証用として追加し、「独立点で検証」を押します。検証点は最適化の残差には入りません。
7. 「保存」で結果・設定・内部較正・全サンプルを同じYAMLに保存します。既定の保存先はユーザー設定ディレクトリ内のcamera-lidar-calibration/extrinsic.yamlです。
8. TFが必要な場合だけ「TFへ反映」を押します。有効な保存結果のみ適用できます。

手動選択した候補を見失った場合は別の物体に切り替えません。新しい位置で再選択するか、「LiDAR自動選択」に戻してください。

GUIの「静止時間」「LiDAR許容幅」「画像許容幅」は「静止条件を適用」で変更できます。この操作では取得済みサンプルを保持します。このPCでは0.4秒・10 mm・4 pxを設定しています。取得には最低3観測が必要なので、処理周期も実際の待ち時間に影響します。

静止判定は窓内の中央値からの揺れ幅（80パーセンタイル）、現在値のずれ、窓の前後の移動量を確認します。短い検出欠落では履歴を保持しますが、対象が見えていない現在フレームでの取得は拒否します。取得するのは中央値に近い実際の画像・LiDARの同期ペアで、異なる時刻の画素と座標を別々に平均する処理は行いません。各サンプルに取得時の安定度も保存します。

取得できない理由、観測数、経過時間、画像とLiDARの揺れ幅をGUIに表示します。球候補が0個のときは、まず候補が検出される必要があります。

画像・LiDARの切断、時刻差超過、静止条件違反、円フィット誤差超過時には取得しません。カメラ画像はLiDAR切断中も表示し、LiDAR点群もカメラの切断・内部較正待ちにかかわらず表示します。カメラ切断時は画像待ち表示になります。LiDARの受信状態・有効点数と、球候補の認識状態を区別して表示します。

設定はユーザー設定ディレクトリのsettings.yamlに保存し、XDG_CONFIG_HOMEを尊重します。共有初期値はconfig/calibration_params.yamlです。HSV、トピック・フレーム、球直径・高さ差、距離、同期・静止・重複判定、目標サンプル数、精度閾値などをGUIで変更できます。較正条件の変更時には、確認後にサンプルを全削除します。

## 推定方法と品質判定

LiDARの無効点除去、隣接点クラスタリング、既知の断面半径を使うロバスト円フィットで球中心を求めます。測距点の平均を中心として扱いません。

画像の認識結果には、輪郭、バウンディングボックス、半径、HSV形状判定に基づく信頼度、画像時刻を保持します。信頼度は分類確率ではありません。既定のcenter_mode=sphereは、歪み補正後の輪郭の接線円錐の軸を使い、輪郭中心と球中心の投影の差を補正します。contourでは輪郭中心を使用します。

正規化画像点とLiDAR平面のホモグラフィで外れ値を除去し、IPPEの複数候補から前方条件と検証精度を満たす解を選び、SciPyのロバスト最小二乗で学習点のみを最適化します。OpenCVのPnPは物体座標からカメラ座標へ変換するRとtを返します。 [OpenCVのPnP仕様](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)

既定の品質条件には次を含みます。GUIで確認・変更できます。

- 学習12点以上、独立検証4点以上
- XY各方向に0.10 m以上、凸包面積0.015 m²以上の広がり
- 画像上の各方向に35 px以上の広がり
- 学習・検証の再投影RMSが2.5 px以下
- 採用点・検証点の最大誤差が6 px以下
- 複数解の識別、前方条件、センサー間距離、ヤコビアンの条件数

結果にはR、t、クォータニオン、RPY、カメラ原点のLiDAR座標、RMS、最大誤差、条件数、各点の誤差と用途・採否、採用数・除外数を保存します。max_errorは除外点も含み、train_maxは採用した学習点だけの最大誤差です。

## 変換方向とTF

結果のrotationとtranslationは次の方向です。

~~~text
p_camera_optical = R * p_lidar + t
~~~

tはカメラ原点のLiDAR座標ではありません。カメラ原点はcamera_origin_in_lidar = −Rᵀtです。rpy_radとquaternion_xyzwはOpenCV側のRを表し、RPYはxyz順・radです。

ROSで親をLiDAR、子をカメラ光学フレームとして配信するときは逆変換を使います。

~~~text
parent = lidar_frame
child = camera_frame
TF.rotation = Rᵀ
TF.translation = -Rᵀ*t
~~~

TFは明示操作です。開始前に/tfと/tf_staticを監査し、同じchildが存在する場合や循環ができる場合は拒否します。同一ユーザーの二重起動は、フレームとROSドメイン単位のロックでも防止します。配信後に競合を検出した場合も停止します。別PCとの同時開始はネットワーク全体で原子的には防げないため、較正TFの配信元は一つにしてください。

静的TFは受信側のキャッシュに残ります。TF停止後や取り付け変更後は、古いTFを受信したノードを再起動してください。既存の同じchildの配信元を勝手に停止・上書きする処理はありません。

~~~bash
ros2 run camera_lidar_calibration calibration_tf_publisher --ros-args   -p extrinsic_file:="$HOME/.config/camera-lidar-calibration/extrinsic.yaml"
~~~

## 較正後の投影

~~~bash
ros2 launch camera_lidar_calibration projection.launch.py
~~~

GUIで最後に保存したファイルを既定として使用します。別のファイルも指定できます。

~~~bash
ros2 launch camera_lidar_calibration projection.launch.py extrinsic_file:=/path/to/extrinsic.yaml
~~~

Control Centerには「較正済みLiDARの画像投影」も追加しています。較正GUIを停止してから起動します。投影だけの起動ではTFを配信しません。

| トピック | 型と意味 |
| --- | --- |
| /camera_lidar/overlay | sensor_msgs/msg/Image。距離別のLiDAR点と紫色の照合済み球クラスタ |
| /camera_lidar/point_cloud | sensor_msgs/msg/PointCloud2。走査面Z=0の点群、LiDARフレーム |
| /ball/position | geometry_msgs/msg/PointStamped。球中心XYZ、LiDARフレーム、走査基準時刻 |
| /ball/distance | std_msgs/msg/Float32。LiDAR原点から球表面までの3次元最短距離 m |
| /ball/detected | std_msgs/msg/Bool。現在の照合成否 |
| /camera_lidar/calibration/status | std_msgs/msg/String。GUI状態とサンプル・結果のJSON |
| /camera_lidar/calibration/preview | sensor_msgs/msg/Image。較正GUIの注釈・投影画像 |
| /camera_lidar/tf_status | std_msgs/msg/String。TF監査・配信状態 |

球の確定には円フィットしたクラスタ、球中心投影、見かけの半径、位置の連続性を使用します。バウンディングボックス内に入っただけの点を球と確定しません。候補が曖昧な場合や見失った場合はdetected=false、距離NaNとなり、新しい位置を配信しません。受信側も検出フラグと時刻を確認してください。

断面の表面距離はサンプルのlidar.scan_surface_distanceに別途保持します。高さ差がある場合、これは/ball/distanceの3次元距離と異なります。

未補正画像にはKと歪み係数を使用します。補正済み画像にはCameraInfoのPとRを使用し、保存変換は元の光学フレームに統一します。基線項を持つステレオPは未対応で、処理を拒否します。

RViz2ではImageに/camera_lidar/overlay、PointCloud2に/camera_lidar/point_cloudを指定します。点群のFixed Frameはlaserにすると単独で確認できます。QoSはBest Effortで受信できます。標準ROSメッセージなのでFoxgloveでも利用できます。

## 保存とオフライン再計算

YAMLには日時、フレーム、内部較正、球直径・高さ差、外部変換、品質指標、検証結果、各サンプルの時刻・画素・XYZ・輪郭・クラスタ測距点を保存します。生画像全体の記録は行いません。

センサーを接続せず「読み込み」→「外部較正実行」で再計算できます。無効な結果でもサンプル保存は可能ですが、TF・投影への利用は拒否します。config/extrinsic.yamlは空の初期例です。

## 主なファイル

| ファイル | 担当 |
| --- | --- |
| ball_detector.py、camera_model.py | 球輪郭、球中心投影補正、内部較正・画像座標 |
| lidar_ball_detector.py | 点群、クラスタ、断面補正、円フィット |
| samples.py | 静止・重複判定、手動・自動取得 |
| extrinsic_solver.py | 平面PnP、外れ値除去、最適化、検証 |
| storage.py、settings.py | 入出力検証、原子的YAML保存、PC固有設定 |
| calibration_node.py | ROS受信と較正操作 |
| gui.py、calibration_gui.py、internal_gui.py | Qt GUI、ROS受信スレッド、既存内部較正の再利用 |
| projection.py、lidar_projection_node.py | 投影、球クラスタ照合、位置配信 |
| calibration_tf_publisher.py | TF逆変換、競合監査、明示配信 |
| launch/、config/、test/ | 起動、初期設定、検証 |

GUIとROS受信は別スレッド、計算・較正処理は別ROSプロセスで動きます。画像・点群履歴、サンプル、GUI送信キューには上限があります。

## テストと未確認事項

~~~bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select camera_lidar_calibration --event-handlers console_direct+
colcon test-result --test-result-base build/camera_lidar_calibration --verbose
~~~

システムPythonにQtがない場合、GUIテストはユーザー用仮想環境で実行します。

~~~bash
QT_QPA_PLATFORM=offscreen   "$HOME/.local/share/ros2-control-center/venv/bin/python" -m pytest   real/src/camera_lidar_calibration/test/test_gui.py -q
~~~

ROS経路テストはドメイン93と模擬センサーを使用します。25点取得、較正、保存、投影・球位置、切断、TF方向、競合拒否を確認します。モーターや実機センサーを使用しません。

実機では、再接続先のカメラ・CameraInfoとLiDARの受信、約12 msの同期を確認しました。実物の球を20〜30か所へ移す較正、取り付け変更後の実機精度、未使用位置での実機誤差、実環境へのTF適用は未確認です。独立位置での投影と誤差を確認してから利用してください。


## 撮影範囲と色認識

「撮影範囲・解像度」からカメラの対応解像度を選べます。このPCのWCMカメラでは、640×480に対して1280×720は縦の範囲をほぼ保ち、横に約1.3倍広く映ることを実写と画像対応で確認しました。現在のPC設定は1280×720です。1920×1080も同程度の範囲ですが、処理負荷を考慮して1280×720を選んでいます。

640×480用の内部較正ファイルは解像度別プロファイルに保持します。新しい解像度用の内部較正を実行し、カメラを停止・再起動してください。保存時の既定名は解像度を含みます。内部較正がない場合も色認識のプレビューは表示しますが、外部較正用サンプルの取得と位置投影は待機します。

カメラ画像の黄色い円は色・形状の候補です。候補が一つ、または面積が次の候補の9倍以上の場合は自動で優先します。面積比は設定で変更でき、0で自動優先を無効にできます。ボールをクリックすると対象を明示選択し、位置と大きさで追跡します。複数の赤い領域がある場合も、選んだ球の認識を維持します。対象が現在の画像で見つからない場合は認識失敗とし、以前の位置を現在の認識結果として返しません。大きく移動して追跡範囲を外れた場合はもう一度クリックしてください。

「ボールの色を選ぶ」を押してから、反射や背景を避けてボールの色がついている部分をクリックするとHSVを設定します。ボタンを再度押すと選択を中止できます。「色マスク表示」で抽出領域を確認できます。色の変更で既存サンプルを削除する場合は、先に確認画面を表示します。

このPCでは、従来の色相下限174が球の一部を除外していたため、下限を160に調整しました。台の赤い模様などが追加候補になる場合は、画像上で球を選択してください。実写の1280×720の67フレームでは、対象選択後の追跡が67フレームで成立しました。照明・距離・背景を変えた場合の一般的な認識率を保証する測定ではありません。

色設定はball_detector.yamlにも出力し、既存のボール認識ノードが次回起動時に利用できるようにしています。既存プログラムのソースや共有のparams.yamlを変更する処理はありません。較正後の位置配信では、複数の画像候補とLiDAR候補を幾何条件で照合して一意な組だけを採用します。
