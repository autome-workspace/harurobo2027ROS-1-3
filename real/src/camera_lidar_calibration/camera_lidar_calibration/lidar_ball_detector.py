import math

import numpy as np
from scipy.optimize import least_squares


def scan_points(scan):
    if (not math.isfinite(scan.angle_min) or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0 or not 0 <= scan.range_min < scan.range_max):
        raise ValueError('LaserScanの角度または距離範囲が無効です。')
    ranges = np.asarray(scan.ranges, float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges > 0) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
    ids = np.flatnonzero(valid)
    return np.column_stack((ranges[ids] * np.cos(angles[ids]), ranges[ids] * np.sin(angles[ids]))), ids


def detect_candidates(scan, settings):
    points, ids = scan_points(scan)
    if not len(points):
        return points, []
    distances = np.linalg.norm(points, axis=1)
    use = (distances >= settings['min_range']) & (distances <= settings['max_range'])
    points_filtered, ids_filtered = points[use], ids[use]
    if len(points_filtered) < settings['min_points']:
        return points, []
    cuts = np.flatnonzero((np.diff(ids_filtered) > settings['max_missing_bins'] + 1) |
                          (np.linalg.norm(np.diff(points_filtered, axis=0), axis=1) > settings['cluster_gap'])) + 1
    groups = list(np.split(np.arange(len(points_filtered)), cuts))
    if (len(groups) > 1 and abs(len(scan.ranges) * abs(scan.angle_increment) - math.tau) < .05
            and (ids_filtered[0] + len(scan.ranges) - ids_filtered[-1]) <= settings['max_missing_bins'] + 1
            and np.linalg.norm(points_filtered[0] - points_filtered[-1]) <= settings['cluster_gap']):
        groups = [np.r_[groups[-1], groups[0]], *groups[1:-1]]
    radius = settings['ball_diameter'] / 2
    height = settings['center_height_offset']
    section = math.sqrt(radius**2 - height**2)
    candidates = []
    for group in groups:
        if len(group) < settings['min_points']:
            continue
        cluster = points_filtered[group]
        span = np.linalg.norm(cluster[0] - cluster[-1])
        if not settings['min_arc_span'] * section <= span <= 2.15 * section:
            continue
        closest = cluster[np.argmin(np.linalg.norm(cluster, axis=1))]
        initial = closest * (1 + section / np.linalg.norm(closest))
        fitted = least_squares(lambda center: np.linalg.norm(cluster - center, axis=1) - section,
                               initial, loss='soft_l1', f_scale=settings['max_fit_rms'], max_nfev=100)
        residual = np.linalg.norm(cluster - fitted.x, axis=1) - section
        rms = float(np.sqrt(np.mean(residual**2)))
        # The center must lie behind the first visible surface, facing away from the scanner.
        scan_surface = float(np.linalg.norm(fitted.x) - section)
        if (not fitted.success or not np.isfinite(fitted.x).all()
                or rms > settings['max_fit_rms']
                or np.max(np.abs(residual)) > settings['max_fit_rms'] * 3
                or abs(scan_surface - np.min(np.linalg.norm(cluster, axis=1))) > settings['max_fit_rms'] * 3):
            continue
        center = [float(fitted.x[0]), float(fitted.x[1]), float(height)]
        candidates.append({
            'center': center, 'radius': float(section), 'fit_rms': rms,
            'points': cluster.tolist(), 'indices': ids_filtered[group].tolist(),
            'surface_distance': float(np.linalg.norm(center) - radius),
            'scan_surface_distance': scan_surface,
        })
    return points, candidates
