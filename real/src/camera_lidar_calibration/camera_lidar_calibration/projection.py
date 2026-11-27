import cv2
import numpy as np

from .extrinsic_solver import project
from .ball_detector import annotate


def overlay(image, points, candidates, detection, result, camera, settings, previous=None):
    detections = detection if isinstance(detection, list) else ([detection] if detection is not None else [])
    output = annotate(image.copy(), detection if isinstance(detection, dict) else None)
    chosen = None
    if result is None or not len(points):
        return output, chosen
    pose = np.r_[result['rvec'], result['translation']]
    xyz = np.column_stack((points, np.zeros(len(points))))
    pixels, depth = project(xyz, pose, camera)
    valid = (np.isfinite(pixels).all(axis=1) & (depth > .001)
             & (pixels[:, 0] >= 0) & (pixels[:, 0] < image.shape[1])
             & (pixels[:, 1] >= 0) & (pixels[:, 1] < image.shape[0]))
    distance = np.linalg.norm(points, axis=1)
    colors = cv2.applyColorMap(np.uint8(np.clip(distance / settings['max_range'], 0, 1) * 255),
                              cv2.COLORMAP_TURBO).reshape(-1, 3)
    for pixel, color in zip(pixels[valid], colors[valid]):
        cv2.circle(output, tuple(np.rint(pixel).astype(int)), 2, color.tolist(), -1)
    matches = []
    for candidate in candidates:
        uv, z = project([candidate['center']], pose, camera)
        transformed = np.asarray(result['rotation']) @ candidate['center'] + result['translation']
        norm = np.linalg.norm(transformed)
        radius = settings['ball_diameter'] / 2
        expected = np.arcsin(min(.999, radius / max(norm, radius + 1e-6)))
        continuity = previous is None or np.linalg.norm(np.array(candidate['center']) - previous) < settings['track_step']
        for detected in detections:
            error = np.linalg.norm(uv[0] - detected['uv'])
            ratio = expected / max(detected['angular_radius'], 1e-6)
            # Require geometric agreement and a fitted sphere cluster, even
            # when several colored regions exist in the camera image.
            if z[0] > .001 and error <= settings['association_pixels'] and .6 <= ratio <= 1.7 and continuity:
                matches.append((candidate, detected))
    if len(matches) == 1:
        chosen, matched_image = matches[0]
        annotate(output, matched_image)
        cluster_xyz = np.column_stack((chosen['points'], np.zeros(len(chosen['points']))))
        projected, depths = project(cluster_xyz, pose, camera)
        for pixel, depth in zip(projected, depths):
            if depth > .001 and 0 <= pixel[0] < image.shape[1] and 0 <= pixel[1] < image.shape[0]:
                cv2.circle(output, tuple(np.rint(pixel).astype(int)), 4, (255, 0, 255), -1)
    return output, chosen
