import copy
import math

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from camera_lidar_calibration.ball_detector import detect_ball
from camera_lidar_calibration.camera_model import CameraModel
from camera_lidar_calibration.extrinsic_solver import solve, project
from camera_lidar_calibration.lidar_ball_detector import detect_candidates
from camera_lidar_calibration.samples import SampleCollector
from camera_lidar_calibration.settings import defaults
from camera_lidar_calibration.storage import bundle, read_bundle, save_bundle, load_valid


def model():
    return CameraModel(640, 480, [[698., 0., 317.], [0., 703., 248.], [0., 0., 1.]],
                       [-.17, -.16, .002, -.001, .28])


def solver_settings():
    settings = defaults()
    settings['center_height_offset'] = -.015
    return settings


def synthetic(seed=4, pose=None):
    camera = model()
    rng = np.random.default_rng(seed)
    points = np.column_stack((rng.uniform(-.45, .45, 30), rng.uniform(-.3, .3, 30),
                              np.full(30, -.015)))
    pose = np.array([.18, -.3, .08, .04, -.03, 1.15]) if pose is None else np.array(pose)
    pixels, _ = project(points, pose, camera)
    pixels += rng.normal(0., .15, pixels.shape)
    pixels[[2, 8]] += [80., -55.]
    samples = [{'id': index + 1, 'role': 'validation' if index % 5 == 4 else 'train',
                'xyz': point.tolist(), 'uv': pixel.tolist()}
               for index, (point, pixel) in enumerate(zip(points, pixels))]
    return camera, pose, samples


def test_planar_robust_solver_and_direction(tmp_path):
    camera, pose, samples = synthetic()
    result = solve(samples, camera, solver_settings())
    assert result['valid'], result['warnings']
    assert result['excluded_samples'] >= 2
    assert result['validation_rms'] < .5
    assert np.linalg.norm(np.array(result['translation']) - pose[3:]) < .004
    assert np.linalg.norm((Rotation.from_matrix(result['rotation']).inv() *
                           Rotation.from_rotvec(pose[:3])).as_rotvec()) < .01
    from camera_lidar_calibration.calibration_tf_publisher import inverse_transform
    inverse_r, inverse_t = inverse_transform(result)
    point = np.array([.2, .1, -.015])
    camera_point = np.array(result['rotation']) @ point + result['translation']
    assert np.allclose(inverse_r @ camera_point + inverse_t, point)
    path = tmp_path / 'extrinsic.yaml'
    settings = solver_settings()
    save_bundle(path, bundle(settings, camera, samples, result))
    saved, loaded_camera = load_valid(path)
    assert saved['result']['valid']
    assert loaded_camera.as_dict() == camera.as_dict()
    assert saved['samples'] == samples
    # Offline re-optimization uses the same full observations.
    again = solve(saved['samples'], loaded_camera, saved['settings'])
    assert np.allclose(again['translation'], result['translation'], atol=1e-7)


def test_validation_and_degeneracy():
    camera, _, samples = synthetic()
    bad = copy.deepcopy(samples)
    for sample in bad:
        if sample['role'] == 'validation':
            sample['uv'][0] += 30
    assert not solve(bad, camera, solver_settings())['valid']
    for sample in samples:
        sample['xyz'][1] = 0.
    with pytest.raises(ValueError, match='一直線'):
        solve(samples, camera, solver_settings())


def test_independent_validation_required():
    camera, _, samples = synthetic()
    for sample in samples:
        sample['role'] = 'train'
    result = solve(samples, camera, solver_settings())
    assert not result['valid']
    assert any('検証点' in warning for warning in result['warnings'])


def test_lidar_section_and_surface_not_mean():
    from sensor_msgs.msg import LaserScan
    scan = LaserScan()
    scan.angle_min = -.4
    scan.angle_increment = .002
    scan.range_min = .05
    scan.range_max = 3.
    settings = defaults()
    settings['center_height_offset'] = -.015
    radius = math.sqrt(.03**2 - .015**2)
    center = np.array([.5, .02])
    ranges = []
    for angle in scan.angle_min + np.arange(400) * scan.angle_increment:
        ray = np.array([math.cos(angle), math.sin(angle)])
        b = ray @ center
        discriminant = b*b - (center @ center - radius**2)
        ranges.append(float(b - math.sqrt(discriminant)) if discriminant >= 0 else float('inf'))
    scan.ranges = ranges
    _, candidates = detect_candidates(scan, settings)
    assert len(candidates) == 1
    candidate = candidates[0]
    assert np.allclose(candidate['center'], [.5, .02, -.015], atol=.0005)
    assert candidate['fit_rms'] < .0001
    mean = np.mean(candidate['points'], axis=0)
    assert np.linalg.norm(mean - center) > .01
    assert np.isclose(candidate['radius'], radius)


def test_spherical_contour_center_correction():
    camera = CameraModel(800, 600, [[650., 0., 400.], [0., 650., 300.], [0., 0., 1.]], [])
    center = np.array([.20, .10, .60])
    radius = .06
    axis = center / np.linalg.norm(center)
    first = np.cross(axis, [0., 1., 0.])
    first /= np.linalg.norm(first)
    second = np.cross(axis, first)
    alpha = math.asin(radius / np.linalg.norm(center))
    theta = np.linspace(0, 2*math.pi, 120)
    rays = (axis[None] * math.cos(alpha) +
            math.sin(alpha) * (np.cos(theta)[:, None] * first + np.sin(theta)[:, None] * second))
    pixels = camera.project(rays)
    image = np.zeros((600, 800, 3), np.uint8)
    cv2.fillPoly(image, [np.rint(pixels).astype(np.int32)], (0, 0, 255))
    settings = defaults()
    settings['min_circularity'] = .6
    detection, count = detect_ball(image, camera, settings)
    assert count == 1
    expected = camera.project([center])[0]
    assert np.linalg.norm(np.array(detection['uv']) - expected) < 1.2
    assert np.linalg.norm(np.array(detection['uv']) - expected) < np.linalg.norm(np.array(detection['contour_uv']) - expected)


def test_rectified_projection_roundtrip():
    rotation = Rotation.from_euler('y', .03).as_matrix()
    camera = CameraModel(640, 480, [[600.,0,320],[0,610.,240],[0,0,1]], [],
                         rectified=True, rectification=rotation,
                         projection=[[590.,0,315.,0],[0,595.,235.,0],[0,0,1.,0]])
    points = np.array([[.1,.2,1.],[-.2,-.1,2.]])
    pixels = camera.project(points)
    assert np.allclose(camera.normalized(pixels), points[:, :2]/points[:, 2:], atol=1e-8)


def test_samples_stationarity_duplicates_and_stale(monkeypatch):
    import camera_lidar_calibration.samples as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    collector = SampleCollector(defaults())
    observation = {'xyz': [.5,0.,0.], 'uv': [320.,240.]}
    for i in range(6):
        clock[0] = i*.2
        collector.observe(observation.copy())
    assert collector.stationary()
    collector.capture()
    clock[0] += 1.1
    collector.observe(observation)
    with pytest.raises(ValueError):
        collector.capture()
    clock[0] += 1.
    with pytest.raises(ValueError, match='古く'):
        collector.capture()


def test_corrupt_transform_rejected(tmp_path):
    camera, _, samples = synthetic()
    result = solve(samples, camera, solver_settings())
    result['rotation'][0][0] += .1
    path = tmp_path/'bad.yaml'
    save_bundle(path, bundle(solver_settings(), camera, samples, result))
    with pytest.raises(ValueError, match='外部変換'):
        read_bundle(path)


def test_recalibration_after_mount_change():
    camera, pose, samples = synthetic(8, [-.25, .2, -.12, -.1, .08, .95])
    result = solve(samples, camera, solver_settings())
    assert result['valid'], result['warnings']
    assert np.linalg.norm(np.array(result['translation']) - pose[3:]) < .004
    assert np.linalg.norm((Rotation.from_matrix(result['rotation']).inv() *
                           Rotation.from_rotvec(pose[:3])).as_rotvec()) < .01


def test_short_stationarity_keeps_pairs_and_rejects_motion(monkeypatch):
    import camera_lidar_calibration.samples as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    settings = defaults()
    settings.update(stationary_duration=.4, stationary_position=.010, stationary_pixels=4.)
    collector = SampleCollector(settings)
    pairs = []
    for index, jitter in enumerate((-.004, .004, 0.)):
        clock[0] = index * .2
        pair = {'xyz': [.5+jitter, 0., 0.], 'uv': [320.+jitter*200, 240.], 'pair_id': index}
        pairs.append(pair)
        collector.observe(pair)
    assert collector.stationary()
    sample = collector.capture()
    assert any(sample['xyz'] == pair['xyz'] and sample['uv'] == pair['uv']
               and sample['pair_id'] == pair['pair_id'] for pair in pairs)
    assert sample['stability']['count'] == 3
    clock[0] = .6
    collector.observe(None)
    assert not collector.stationary()
    assert collector.history
    clock[0] = .8
    collector.observe({'xyz': [.56, 0., 0.], 'uv': [340.,240.]})
    assert not collector.stationary()
    clock[0] = 2.
    collector.observe(None)
    assert not collector.history


def test_camera_selection_tracks_ball_among_small_red_marks():
    from camera_lidar_calibration.ball_detector import CameraTracker, find_balls
    settings = defaults()
    settings['hsv_lower'] = [160,70,60]
    color = cv2.cvtColor(np.uint8([[[168,190,220]]]), cv2.COLOR_HSV2BGR)[0,0].tolist()
    image = np.zeros((480,640,3),np.uint8)
    cv2.circle(image,(300,240),45,color,-1)
    cv2.circle(image,(300,310),8,color,-1)
    candidates = find_balls(image,None,settings)
    assert len(candidates)==2
    tracker=CameraTracker()
    automatic,_=tracker.detect(image,None,settings)
    assert automatic is not None and automatic['radius']>40
    tracker.select([300,240],candidates)
    detection,_=tracker.detect(image,None,settings)
    assert detection is not None and detection['radius']>40
    assert not detection['geometry_valid']
    assert detection['ray']==[0.,0.,0.]
    missing=np.zeros_like(image)
    cv2.circle(missing,(300,310),8,color,-1)
    detection,_=tracker.detect(missing,None,settings)
    assert detection is None


def test_color_picker_handles_red_wrap_and_rejects_glare():
    from camera_lidar_calibration.ball_detector import pick_hsv
    image=np.zeros((40,40,3),np.uint8)
    image[:,:20]=cv2.cvtColor(np.uint8([[[178,190,220]]]),cv2.COLOR_HSV2BGR)[0,0]
    image[:,20:]=cv2.cvtColor(np.uint8([[[2,190,220]]]),cv2.COLOR_HSV2BGR)[0,0]
    low,high=pick_hsv(image,20,20)
    assert low[0]>high[0]
    assert low[1]<=190 and low[2]<=220
    with pytest.raises(ValueError,match='反射'):
        pick_hsv(np.full((40,40,3),255,np.uint8),20,20)


def test_projection_resolves_multiple_image_candidates():
    from camera_lidar_calibration.ball_detector import find_balls
    from camera_lidar_calibration.projection import overlay
    camera=model()
    image=np.zeros((480,640,3),np.uint8)
    cv2.circle(image,(317,248),21,(0,0,255),-1)
    cv2.circle(image,(410,340),8,(0,0,255),-1)
    detections=find_balls(image,camera,defaults())
    assert len(detections)==2
    candidate={'center':[.5,0.,0.], 'points':[[.47,0.],[.475,.01]],
               'surface_distance':.47}
    result={'rvec':[0.,0.,0.],'rotation':np.eye(3).tolist(),'translation':[-.5,0.,1.]}
    _,selected=overlay(image,np.array([[.47,0.],[.475,.01]]),[candidate],detections,
                       result,camera,defaults())
    assert selected is candidate

def test_old_bundle_accepts_new_acquisition_defaults(tmp_path):
    camera, _, samples = synthetic()
    result = solve(samples, camera, solver_settings())
    settings = solver_settings()
    settings.pop('image_dominance_ratio')
    path = tmp_path/'old-schema.yaml'
    save_bundle(path, bundle(settings, camera, samples, result))
    saved, _ = read_bundle(path)
    assert saved['settings']['image_dominance_ratio'] == 9.


@pytest.mark.parametrize('pose', [
    [.18, -.3, .08, .04, -.03, 1.15],
    [-.25, .15, -.12, -.05, .08, 1.3],
])
def test_five_sample_calibration_and_validation(tmp_path, pose):
    from camera_lidar_calibration.settings import validate
    camera = model()
    settings = solver_settings() | {'min_train_samples': 4, 'min_validation_samples': 1,
                                    'target_samples': 5}
    validate(settings)
    points = np.array([[-.22, -.16, -.015], [.23, -.15, -.015],
                       [.24, .17, -.015], [-.21, .18, -.015], [.02, -.01, -.015]])
    pixels, _ = project(points, pose, camera)
    samples = [{'id': i+1, 'role': 'validation' if i == 4 else 'train',
                'xyz': point.tolist(), 'uv': pixel.tolist()}
               for i, (point, pixel) in enumerate(zip(points, pixels))]
    result = solve(samples, camera, settings)
    assert result['valid'], result['warnings']
    assert np.allclose(result['translation'], pose[3:], atol=1e-5)
    assert result['used_samples'] == 4 and result['validation_samples'] == 1
    path = tmp_path / 'five.yaml'
    save_bundle(path, bundle(settings, camera, samples, result))
    assert load_valid(path)[0]['result']['valid']
    # Four fitted points alone cannot establish independent validity.
    assert not solve(samples[:4], camera, settings)['valid']
    bad = copy.deepcopy(samples)
    bad[-1]['uv'][0] += 30
    assert not solve(bad, camera, settings)['valid']
    line = copy.deepcopy(samples)
    for sample in line:
        sample['xyz'][1] = 0.
    with pytest.raises(ValueError, match='一直線'):
        solve(line, camera, settings)
    narrow = copy.deepcopy(samples)
    for sample in narrow:
        sample['xyz'][0] *= .01
        sample['xyz'][1] *= .01
    with pytest.raises(ValueError, match='分布'):
        solve(narrow, camera, settings)
    with pytest.raises(ValueError):
        validate(settings | {'min_train_samples': 3})


@pytest.mark.parametrize('start,increment', [(-math.pi, math.tau/1440),
                                            (0., math.tau/1440),
                                            (math.pi, -math.tau/1440)])
def test_lidar_sector_filters_points_and_candidates(start, increment):
    from types import SimpleNamespace
    from camera_lidar_calibration.settings import validate
    angles = start + np.arange(1440)*increment
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    settings = solver_settings()
    radius = math.sqrt(.03**2-.015**2)
    ranges = np.full(1440, np.inf)
    for center in (np.array([.5, 0.]), np.array([-.5, 0.])):
        b = rays@center
        discriminant = b*b-(center@center-radius**2)
        hit = (b>0)&(discriminant>=0)
        ranges[hit] = np.minimum(ranges[hit], b[hit]-np.sqrt(discriminant[hit]))
    scan = SimpleNamespace(angle_min=start, angle_increment=increment,
                           range_min=.05, range_max=3., ranges=ranges.tolist())
    original = list(scan.ranges)
    _, all_candidates = detect_candidates(scan, settings)
    assert len(all_candidates) == 2
    settings |= {'lidar_sector_enabled': True, 'lidar_sector_center_deg': 180.,
                 'lidar_sector_width_deg': 180.}
    validate(settings)
    points, candidates = detect_candidates(scan, settings)
    assert len(candidates) == 1 and candidates[0]['center'][0] < 0
    assert np.all(points[:, 0] <= 1e-10)
    ids = candidates[0]['indices']
    reconstructed = ranges[ids, None]*rays[ids]
    assert np.allclose(reconstructed, candidates[0]['points'])
    settings['lidar_sector_center_deg'] = 0.
    points, candidates = detect_candidates(scan, settings)
    assert len(candidates) == 1 and candidates[0]['center'][0] > 0
    settings['lidar_sector_width_deg'] = 360.
    assert len(detect_candidates(scan, settings)[1]) == 2
    assert scan.ranges == original
    with pytest.raises(ValueError):
        validate(settings | {'lidar_sector_width_deg': 0.})
    with pytest.raises(ValueError):
        validate(settings | {'lidar_sector_center_deg': 181.})


def test_three_point_sphere_is_optional_and_fitted():
    from types import SimpleNamespace
    from camera_lidar_calibration.settings import validate
    settings = solver_settings()
    radius = math.sqrt(.03**2-.015**2)
    angles = np.arange(360)*math.pi/180
    rays = np.column_stack((np.cos(angles), np.sin(angles)))
    center = .5*np.array([math.cos(math.radians(191)), math.sin(math.radians(191))])
    ranges = np.full(360, np.inf)
    for i in [189, 191, 193]:
        b = rays[i]@center
        ranges[i] = b-math.sqrt(b*b-(center@center-radius**2))
    scan = SimpleNamespace(angle_min=0., angle_increment=math.pi/180,
                           range_min=.05, range_max=3., ranges=ranges.tolist())
    assert not detect_candidates(scan, settings)[1]
    settings['min_points'] = 3
    validate(settings)
    candidates = detect_candidates(scan, settings)[1]
    assert len(candidates) == 1
    assert np.allclose(candidates[0]['center'][:2], center, atol=1e-4)
    assert candidates[0]['fit_rms'] < 1e-5
    # Two returns are never enough for a sphere candidate.
    scan.ranges[189] = float('inf')
    assert not detect_candidates(scan, settings)[1]
    with pytest.raises(ValueError):
        validate(settings | {'min_points': 2})


def test_inconsistent_correspondences_report_sample_ids():
    camera = model()
    settings = solver_settings() | {'min_train_samples': 4, 'min_validation_samples': 1,
                                    'target_samples': 5}
    points = np.array([[-.22, -.16, -.015], [.23, -.15, -.015],
                       [.24, .17, -.015], [-.21, .18, -.015]])
    pixels, _ = project(points, [.18, -.3, .08, .04, -.03, 1.15], camera)
    pixels[0] += [120., -100.]
    samples = [{'id': i+1, 'role': 'train', 'xyz': point.tolist(), 'uv': pixel.tolist()}
               for i, (point, pixel) in enumerate(zip(points, pixels))]
    with pytest.raises(ValueError, match='ずれが大きい点：ID'):
        solve(samples, camera, settings)
