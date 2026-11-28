"""Use a validated saved extrinsic without publishing a competing camera TF."""
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'real/src/camera_lidar_calibration'))
from camera_lidar_calibration.storage import load_valid


def observed_minimum(data):
    approved = {sample['id'] for sample in data['result']['sample_errors']
                if sample['used'] or sample['role'] == 'validation'}
    return min(float(np.linalg.norm(sample['xyz']) - data['ball_diameter'] / 2)
               for sample in data['samples'] if sample['id'] in approved)


def apply_calibration(parameters, geometry, path):
    data, camera = load_valid(path)
    settings, result = data['settings'], data['result']
    orientation = geometry.get('lidar_orientation_rad')
    if orientation is None or abs(orientation[0]) > .01 or abs(orientation[1]) > .01:
        raise ValueError('較正済み接近には水平なLiDARの取り付け位置と向きが必要です。')
    localizer = parameters['ball_localizer']['ros__parameters']
    localizer.update(
        association_mode='calibrated', use_saved_extrinsic=True,
        calibrated_camera_frame=data['camera_frame'], calibrated_lidar_frame=data['lidar_frame'],
        camera_rotation_in_lidar=np.asarray(result['rotation']).T.ravel().tolist(),
        camera_origin_in_lidar=result['camera_origin_in_lidar'],
        ball_radius=data['ball_diameter'] / 2,
        ball_center_z=geometry['lidar_position_m'][2] + data['center_height_offset'],
        sphere_surface_distance=True,
        min_points=settings['min_points'], cluster_gap=settings['cluster_gap'],
        max_missing_bins=settings['max_missing_bins'], fit_error=settings['max_fit_rms'] * 3,
        angular_gate=math.atan(settings['association_pixels'] / min(camera.matrix[0, 0], camera.matrix[1, 1])),
        max_range=settings['max_range'], max_stamp_skew=settings['sync_tolerance'],
        scan_stamp_is_end=settings['scan_stamp_is_end'],
        lidar_sector_enabled=settings['lidar_sector_enabled'],
        lidar_sector_center_deg=settings['lidar_sector_center_deg'],
        lidar_sector_width_deg=settings['lidar_sector_width_deg'])
    box = geometry.get('lidar_self_filter_box_m')
    if box is not None:
        box = np.asarray(box, float)
        if (box.shape != (4,) or not np.isfinite(box).all()
                or not -geometry['body_length_m']/2 < box[0] < box[1] < geometry['body_length_m']/2
                or not -geometry['body_width_m']/2 < box[2] < box[3] < geometry['body_width_m']/2):
            raise ValueError('機体反射の除外範囲は車体内部の確認済み範囲だけ指定してください。')
        localizer.update(self_filter_enabled=True, self_filter_box=box.tolist())
    else:
        localizer.update(self_filter_enabled=False)
    control = parameters['approach_controller']['ros__parameters']
    minimum = observed_minimum(data)
    configured_minimum = control.get('validated_min_range', 0.)
    if configured_minimum > 0 and configured_minimum < minimum - 1e-9:
        raise ValueError(f'保存較正の確認距離{math.ceil(minimum*1000)}mmより近い距離は許可されていません。')
    control.update(association_mode='calibrated', camera_align_enabled=False,
                   require_camera_during_tracking=True,
                   max_vx=min(.03, control.get('max_vx', .03)),
                   max_vy=min(.03, control.get('max_vy', .03)))
    return data
