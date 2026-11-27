from datetime import datetime, timezone

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def project(points, pose, camera):
    rotation = Rotation.from_rotvec(pose[:3]).as_matrix()
    transformed = np.asarray(points) @ rotation.T + pose[3:]
    return camera.project(transformed), transformed[:, 2]


def errors(samples, pose, camera):
    if not samples:
        return np.empty(0)
    pixels, depth = project([sample['xyz'] for sample in samples], pose, camera)
    values = np.linalg.norm(pixels - np.array([sample['uv'] for sample in samples]), axis=1)
    return np.where(depth > .001, values, 1e6)


def distribution(samples, settings):
    points = np.array([sample['xyz'][:2] for sample in samples], float)
    pixels = np.array([sample['uv'] for sample in samples], float)
    if len(points) < settings['min_train_samples']:
        raise ValueError(f"学習用サンプルを{settings['min_train_samples']}個以上取得してください。")
    singular = np.linalg.svd(points - points.mean(axis=0), compute_uv=False)
    if singular[1] < 1e-8 or singular[1] / singular[0] < .05:
        raise ValueError('サンプルがほぼ一直線です。左右と前後の両方向に広げてください。')
    hull_area = float(ConvexHull(points).volume)
    if np.min(np.ptp(points, axis=0)) < settings['min_axis_span'] or hull_area < settings['min_hull_area']:
        raise ValueError('サンプル位置の分布が不足しています。近距離・遠距離・左右を追加してください。')
    if np.min(np.ptp(pixels, axis=0)) < settings['min_pixel_span']:
        raise ValueError('画像上のサンプル分布が不足しています。')
    return hull_area


def solve(samples, camera, settings):
    training = [sample for sample in samples if sample['role'] == 'train']
    validation = [sample for sample in samples if sample['role'] == 'validation']
    distribution(training, settings)
    points = np.asarray([sample['xyz'] for sample in training], float)
    pixels = np.asarray([sample['uv'] for sample in training], float)
    if not np.isfinite(points).all() or not np.isfinite(pixels).all():
        raise ValueError('サンプルに非有限値が含まれています。')
    if np.ptp(points[:, 2]) > 1e-6:
        raise ValueError('IPPEには同一高さの対応点が必要です。高さを変えた場合はサンプルを取り直してください。')
    if not np.allclose(points[:, 2], settings['center_height_offset'], atol=1e-6):
        raise ValueError('サンプルの高さと設定の高さ差が一致しません。')
    plane_height = float(points[0, 2])
    planar = points.copy()
    planar[:, 2] = 0.
    normalized = camera.normalized(pixels)
    # Robust planar consensus is computed before IPPE. Undistortion keeps the
    # homography assumption valid for wide fields and nonzero lens distortion.
    focal = float((camera.matrix[0, 0] + camera.matrix[1, 1]) / 2)
    _, mask = cv2.findHomography(planar[:, :2], normalized, cv2.RANSAC,
                                settings['ransac_pixels'] / focal, maxIters=3000, confidence=.999)
    if mask is None:
        raise ValueError('平面対応を推定できません。誤対応や分布を確認してください。')
    consensus = mask.ravel().astype(bool)
    if consensus.sum() < settings['min_train_samples']:
        raise ValueError('外れ値除去後の学習サンプルが不足しています。')
    answer = cv2.solvePnPGeneric(planar[consensus], normalized[consensus], np.eye(3), None,
                                flags=cv2.SOLVEPNP_IPPE)
    if not answer[0]:
        raise ValueError('平面PnPを解けませんでした。')
    fits = []
    def residual(pose):
        projected, depths = project(points[consensus], pose, camera)
        result = projected - pixels[consensus]
        result[depths <= .001] = 1e5
        return result.ravel()
    for rvec, tvec in zip(answer[1], answer[2]):
        rotation = Rotation.from_rotvec(rvec.ravel()).as_matrix()
        translation = tvec.ravel() - rotation @ np.array([0., 0., plane_height])
        initial = np.r_[rvec.ravel(), translation]
        optimized = least_squares(residual, initial, loss='soft_l1', f_scale=settings['ransac_pixels'],
                                  max_nfev=600, x_scale='jac')
        pose = optimized.x
        projected, depths = project(points, pose, camera)
        if (not optimized.success or np.min(depths) <= .001
                or np.linalg.norm(pose[3:]) > settings['max_translation']):
            continue
        training_errors = errors(training, pose, camera)
        inliers = consensus & (training_errors <= settings['max_error_pixels'])
        if inliers.sum() < settings['min_train_samples']:
            continue
        # Refit only the training consensus. Validation points never enter the optimizer.
        inlier_points, inlier_pixels = points[inliers], pixels[inliers]
        def final_residual(value):
            projected, depths = project(inlier_points, value, camera)
            output = projected - inlier_pixels
            output[depths <= .001] = 1e5
            return output.ravel()
        final = least_squares(final_residual, pose, loss='soft_l1',
                              f_scale=settings['ransac_pixels'], max_nfev=600, x_scale='jac')
        if not final.success:
            continue
        pose = final.x
        _, final_depths = project(points, pose, camera)
        if np.min(final_depths) <= .001 or np.linalg.norm(pose[3:]) > settings['max_translation']:
            continue
        all_errors = errors(samples, pose, camera)
        train_errors = errors(training, pose, camera)
        validation_errors = errors(validation, pose, camera)
        if np.max(train_errors[inliers]) > settings['max_error_pixels']:
            continue
        norms = np.maximum(np.linalg.norm(final.jac, axis=0), 1e-12)
        singular = np.linalg.svd(final.jac / norms, compute_uv=False)
        condition = float(singular[0] / max(singular[-1], 1e-12))
        train_rms = float(np.sqrt(np.mean(train_errors[inliers]**2)))
        validation_rms = float(np.sqrt(np.mean(validation_errors**2))) if len(validation_errors) else None
        score = train_rms + (validation_rms if validation_rms is not None else 0)
        if any(np.linalg.norm(pose - old['pose']) < 1e-4 for old in fits):
            continue
        fits.append(dict(pose=pose, condition=condition, train_rms=train_rms,
                         validation_rms=validation_rms, errors=all_errors,
                         inliers=inliers, score=score))
    if not fits:
        raise ValueError('前方条件・物理範囲・再投影精度を満たす解がありません。')
    fits.sort(key=lambda item: item['score'])
    best = fits[0]
    warnings = []
    if len(fits) > 1 and fits[1]['score'] - best['score'] < settings['ambiguity_pixels']:
        warnings.append('平面PnPの複数解を区別できません。視野の端や異なる距離の検証点を追加してください。')
    if best['condition'] > settings['max_condition']:
        warnings.append('解の条件数が大きく、姿勢推定が不安定です。')
    if best['train_rms'] > settings['max_rms_pixels']:
        warnings.append('学習点の再投影RMSが許容値を超えています。')
    validation_errors = errors(validation, best['pose'], camera)
    if len(validation) < settings['min_validation_samples']:
        warnings.append(f"独立した検証点を{settings['min_validation_samples']}個以上取得してください。")
    elif (best['validation_rms'] > settings['max_rms_pixels']
          or np.max(validation_errors) > settings['max_error_pixels']):
        warnings.append('独立した検証点の投影誤差が許容値を超えています。')
    pose = best['pose']
    rotation = Rotation.from_rotvec(pose[:3])
    train_index = 0
    sample_errors = []
    for sample, error in zip(samples, best['errors']):
        used = False
        if sample['role'] == 'train':
            used = bool(best['inliers'][train_index])
            train_index += 1
        sample_errors.append({'id': sample['id'], 'role': sample['role'],
                              'error_pixels': float(error), 'used': used})
    return {
        'valid': not warnings, 'warnings': warnings,
        'calibrated_at': datetime.now(timezone.utc).isoformat(),
        'direction': 'lidar_to_camera_optical', 'rotation': rotation.as_matrix().tolist(),
        'translation': pose[3:].tolist(), 'quaternion_xyzw': rotation.as_quat().tolist(),
        'rpy_rad': rotation.as_euler('xyz').tolist(), 'rvec': pose[:3].tolist(),
        'camera_origin_in_lidar': (-rotation.as_matrix().T @ pose[3:]).tolist(),
        'tf_rotation': rotation.as_matrix().T.tolist(),
        'tf_quaternion_xyzw': rotation.inv().as_quat().tolist(),
        'train_rms': best['train_rms'],
        'train_max': float(np.max(errors(training, pose, camera)[best['inliers']])),
        'validation_rms': best['validation_rms'],
        'validation_max': float(np.max(validation_errors)) if len(validation_errors) else None,
        'max_error': float(np.max(best['errors'])), 'condition': best['condition'],
        'used_samples': int(best['inliers'].sum()),
        'excluded_samples': int(len(training) - best['inliers'].sum()),
        'validation_samples': len(validation), 'sample_errors': sample_errors,
    }


def validate_result(result, samples, camera, settings):
    pose = np.r_[result['rvec'], result['translation']]
    validation = [sample for sample in samples if sample['role'] == 'validation']
    values = errors(validation, pose, camera)
    passed = (len(values) >= settings['min_validation_samples']
              and float(np.sqrt(np.mean(values**2))) <= settings['max_rms_pixels']
              and float(np.max(values)) <= settings['max_error_pixels'])
    return {'valid': bool(passed), 'count': len(values),
            'rms': float(np.sqrt(np.mean(values**2))) if len(values) else None,
            'max': float(np.max(values)) if len(values) else None}
