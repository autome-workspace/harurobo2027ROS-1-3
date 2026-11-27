import json
import math
import os
from pathlib import Path
import tempfile

import yaml


def user_directory():
    return Path(os.environ.get('XDG_CONFIG_HOME', Path.home() / '.config')) / 'camera-lidar-calibration'


def defaults():
    path = Path(__file__).resolve().parents[1] / 'config/calibration_params.yaml'
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('camera_lidar_calibration')) / 'config/calibration_params.yaml'
    return yaml.safe_load(path.read_text())['camera_lidar_calibration']['ros__parameters']


def validate(settings):
    expected = defaults()
    if set(settings) != set(expected):
        raise ValueError('設定項目が不足しているか、不明な項目があります。')
    for key, value in settings.items():
        reference = expected[key]
        if isinstance(reference, bool):
            good = type(value) is bool
        elif isinstance(reference, int):
            good = type(value) is int
        elif isinstance(reference, float):
            good = type(value) in (float, int) and math.isfinite(value)
        else:
            good = isinstance(value, type(reference))
        if not good:
            raise ValueError(f'{key} の型が無効です。')
    positive = (
        'ball_diameter', 'min_area', 'min_range', 'max_range', 'cluster_gap', 'max_fit_rms',
        'min_arc_span', 'sync_tolerance', 'stale_timeout', 'stationary_duration',
        'stationary_position', 'stationary_pixels', 'sample_interval', 'duplicate_distance',
        'min_axis_span', 'min_hull_area', 'min_pixel_span', 'ransac_pixels',
        'max_rms_pixels', 'max_error_pixels', 'max_translation', 'max_condition',
        'association_pixels', 'track_step', 'processing_hz',
    )
    if any(settings[key] <= 0 for key in positive):
        raise ValueError('距離・時間・許容誤差は正の値にしてください。')
    if not 0 < settings['min_range'] < settings['max_range']:
        raise ValueError('認識距離の上下限が無効です。')
    if abs(settings['center_height_offset']) >= settings['ball_diameter'] / 2:
        raise ValueError('走査面が球に交差しません。高さ差は球の半径未満にしてください。')
    if settings['image_dominance_ratio'] != 0 and settings['image_dominance_ratio'] < 1:
        raise ValueError('候補面積比は0、または1以上にしてください。')
    if settings['center_mode'] not in ('sphere', 'contour'):
        raise ValueError('center_mode は sphere または contour です。')
    for key in ('hsv_lower', 'hsv_upper'):
        value = settings[key]
        if len(value) != 3 or any(type(v) is not int or not 0 <= v <= limit
                                  for v, limit in zip(value, (179, 255, 255))):
            raise ValueError('HSVはHが0–179、S/Vが0–255の整数です。')
    if any(settings['hsv_lower'][i] > settings['hsv_upper'][i] for i in (1, 2)):
        raise ValueError('HSVのS/V上下限が逆転しています。')
    if not (0 < settings['min_circularity'] <= 1 and 0 < settings['min_solidity'] <= 1):
        raise ValueError('円形度・充実度は0より大きく1以下にしてください。')
    if not (4 <= settings['min_points'] <= 1000 and 0 <= settings['max_missing_bins'] <= 20
            and 12 <= settings['min_train_samples'] <= settings['max_samples'] <= 500
            and 4 <= settings['min_validation_samples'] <= settings['max_samples']
            and 1 <= settings['validation_every'] <= 100
            and 16 <= settings['target_samples'] <= settings['max_samples']
            and .2 <= settings['processing_hz'] <= 15):
        raise ValueError('サンプル数・点数・処理周期が範囲外です。')
    if settings['min_train_samples'] + settings['min_validation_samples'] > settings['max_samples']:
        raise ValueError('最大学習・検証サンプル数の合計が取得上限を超えています。')
    for key in ('camera_frame', 'lidar_frame'):
        if not settings[key] or settings[key].startswith('/') or any(c.isspace() for c in settings[key]):
            raise ValueError('フレーム名は先頭スラッシュや空白を含めず指定してください。')
    if settings['camera_frame'] == settings['lidar_frame']:
        raise ValueError('カメラとLiDARには別のフレーム名を指定してください。')
    for key in ('image_topic', 'info_topic', 'scan_topic'):
        if not settings[key].startswith('/') or any(c.isspace() for c in settings[key]):
            raise ValueError('トピックは絶対名で指定してください。')
    return settings


def atomic_yaml(path, data):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    # Also rejects NaN, infinity and non-serializable message objects.
    json.dumps(data, allow_nan=False)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        try:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
