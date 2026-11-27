import copy
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from .camera_model import CameraModel
from .settings import atomic_yaml, defaults, validate


def bundle(settings, camera, samples, result):
    return {'schema_version': 1, 'settings': copy.deepcopy(settings),
            'camera_frame': settings['camera_frame'], 'lidar_frame': settings['lidar_frame'],
            'camera': camera.as_dict() if camera else None,
            'ball_diameter': settings['ball_diameter'],
            'center_height_offset': settings['center_height_offset'],
            'samples': copy.deepcopy(samples), 'result': copy.deepcopy(result)}


def read_bundle(path):
    try:
        data = yaml.safe_load(Path(path).expanduser().read_text())
    except yaml.YAMLError as error:
        raise ValueError('較正YAMLを読み取れません。') from error
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('較正ファイルの形式が未対応です。')
    required = {'settings', 'camera_frame', 'lidar_frame', 'camera', 'samples', 'result',
                'ball_diameter', 'center_height_offset'}
    if not required <= data.keys():
        raise ValueError('較正GUIで保存した結果とサンプルを指定してください。')
    # Additive acquisition settings must not prevent old sample bundles from loading.
    data['settings'] = defaults() | data['settings']
    validate(data['settings'])
    camera = CameraModel.from_dict(data['camera']) if data['camera'] else None
    samples = data['samples']
    if samples and camera is None:
        raise ValueError('サンプルに対応する内部較正がありません。')
    if not isinstance(samples, list) or len(samples) > data['settings']['max_samples']:
        raise ValueError('保存サンプル数が無効です。')
    if (data['camera_frame'] != data['settings']['camera_frame']
            or data['lidar_frame'] != data['settings']['lidar_frame']
            or data['ball_diameter'] != data['settings']['ball_diameter']
            or data['center_height_offset'] != data['settings']['center_height_offset']):
        raise ValueError('保存されたフレーム・球の寸法・高さ差が一致しません。')
    ids = set()
    for sample in samples:
        if (type(sample['id']) is not int or sample['id'] <= 0 or sample['id'] in ids or sample['role'] not in ('train', 'validation')
                or np.asarray(sample['xyz']).shape != (3,) or np.asarray(sample['uv']).shape != (2,)
                or not np.isfinite(sample['xyz']).all() or not np.isfinite(sample['uv']).all()
                or abs(sample['xyz'][2] - data['settings']['center_height_offset']) > 1e-6):
            raise ValueError('保存サンプルが無効です。')
        ids.add(sample['id'])
    result = data['result']
    if result is not None:
        rotation = np.asarray(result['rotation'], float)
        translation = np.asarray(result['translation'], float)
        if (rotation.shape != (3, 3) or translation.shape != (3,)
                or not np.isfinite(translation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)
                or not np.allclose(Rotation.from_rotvec(result['rvec']).as_matrix(), rotation, atol=1e-6)
                or not np.allclose(Rotation.from_quat(result['quaternion_xyzw']).as_matrix(), rotation, atol=1e-6)):
            raise ValueError('保存された外部変換が無効です。')
        if (data['camera_frame'] != data['settings']['camera_frame']
                or data['lidar_frame'] != data['settings']['lidar_frame']
                or result['direction'] != 'lidar_to_camera_optical'):
            raise ValueError('変換方向・フレーム名が無効です。')
    return data, camera


def load_valid(path):
    data, camera = read_bundle(path)
    if camera is None or not data['result'] or not data['result']['valid']:
        raise ValueError('有効な較正結果が保存されていません。')
    from .extrinsic_solver import distribution, errors, validate_result
    training = [sample for sample in data['samples'] if sample['role'] == 'train']
    distribution(training, data['settings'])
    pose = np.r_[data['result']['rvec'], data['result']['translation']]
    train_errors = errors(training, pose, camera)
    inliers = train_errors <= data['settings']['max_error_pixels']
    if (inliers.sum() < data['settings']['min_train_samples']
            or np.sqrt(np.mean(train_errors[inliers]**2)) > data['settings']['max_rms_pixels']
            or np.linalg.norm(pose[3:]) > data['settings']['max_translation']
            or data['result']['warnings']
            or data['result']['condition'] > data['settings']['max_condition']):
        raise ValueError('保存較正の学習点・配置・安定性の条件を満たしません。')
    validation = validate_result(data['result'], data['samples'], camera, data['settings'])
    if not validation['valid']:
        raise ValueError('保存された検証点で再検証できませんでした。')
    return data, camera


def save_bundle(path, data):
    atomic_yaml(path, data)
