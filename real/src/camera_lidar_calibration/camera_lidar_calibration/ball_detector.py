"""Same HSV wrapping, hull and circularity criteria as the existing ball detector."""
import math

import cv2
import numpy as np


def make_mask(image, settings):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    low, high = settings['hsv_lower'], settings['hsv_upper']
    if low[0] <= high[0]:
        mask = cv2.inRange(hsv, tuple(low), tuple(high))
    else:
        mask = cv2.inRange(hsv, tuple(low), (179, *high[1:]))
        mask |= cv2.inRange(hsv, (0, *low[1:]), tuple(high))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask


def pick_hsv(image, u, v):
    x, y = round(u), round(v)
    if not 0 <= x < image.shape[1] or not 0 <= y < image.shape[0]:
        raise ValueError('画像の中をクリックしてください。')
    patch = image[max(0, y-4):y+5, max(0, x-4):x+5]
    pixels = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    pixels = pixels[(pixels[:, 1] >= 50) & (pixels[:, 2] >= 40)]
    if len(pixels) < 16:
        raise ValueError('白い反射部分や暗部を避け、ボールの色がある部分を選んでください。')
    angles = pixels[:, 0] * (2 * math.pi / 180)
    center = math.atan2(np.sin(angles).mean(), np.cos(angles).mean()) * 180 / (2 * math.pi) % 180
    offsets = (pixels[:, 0] - center + 90) % 180 - 90
    low, high = np.quantile(offsets, [.05, .95])
    if high - low > 30:
        raise ValueError('複数の色が混ざっています。ボールの内側を選んでください。')
    lower = [int(math.floor(center + low - 12)) % 180,
             int(min(70, max(40, np.quantile(pixels[:, 1], .05) - 35))),
             int(min(60, max(30, np.quantile(pixels[:, 2], .05) - 50)))]
    upper = [int(math.ceil(center + high + 12)) % 180, 255, 255]
    return lower, upper


def find_balls(image, camera, settings):
    mask = make_mask(image, settings)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True):
        area = cv2.contourArea(contour)
        if area < settings['min_area']:
            continue
        hull = cv2.convexHull(contour)
        perimeter = cv2.arcLength(hull, True)
        hull_area = cv2.contourArea(hull)
        if perimeter <= 0 or hull_area <= 0:
            continue
        circularity = 4 * math.pi * hull_area / perimeter**2
        solidity = area / hull_area
        if circularity < settings['min_circularity'] or solidity < settings['min_solidity']:
            continue
        (u, v), radius = cv2.minEnclosingCircle(hull)
        if min(u, image.shape[1] - 1 - u, v, image.shape[0] - 1 - v) <= radius:
            continue
        corrected = np.array([u, v], dtype=float)
        if camera is None:
            candidates.append({
                'uv': [float(u), float(v)], 'contour_uv': [float(u), float(v)],
                'radius': float(radius), 'area': float(area), 'bbox': list(map(int, cv2.boundingRect(hull))),
                'contour': hull[:, 0].tolist(), 'confidence': float(min(1., circularity) * solidity),
                'ray': [0., 0., 0.], 'angular_radius': 0., 'geometry_valid': False,
            })
            continue
        normalized = camera.normalized(hull[:, 0])
        if not np.isfinite(normalized).all():
            continue
        angular_radius = None
        if settings['center_mode'] == 'sphere':
            if len(normalized) < 5:
                continue
            # A sphere's tangent cone has its axis through the true 3-D center.
            # The ellipse's 2-D contour center is generally displaced from that axis.
            (cx, cy), (width, height), degrees = cv2.fitEllipse(normalized.astype(np.float32))
            if min(width, height) <= 1e-8:
                continue
            theta = math.radians(degrees)
            rotation = np.array([[math.cos(theta), -math.sin(theta)],
                                 [math.sin(theta), math.cos(theta)]])
            matrix = rotation @ np.diag([4 / width**2, 4 / height**2]) @ rotation.T
            center = np.array([cx, cy])
            conic = np.empty((3, 3))
            conic[:2, :2] = matrix
            conic[:2, 2] = conic[2, :2] = -matrix @ center
            conic[2, 2] = center @ matrix @ center - 1
            values, vectors = np.linalg.eigh(conic)
            if not (values[0] < 0 < values[1]):
                continue
            ray = vectors[:, 0]
            if ray[2] < 0:
                ray = -ray
            corrected = camera.project(ray.reshape(1, 3))[0]
            angular_radius = math.atan(math.sqrt(-values[0] / np.mean(values[1:])))
        else:
            ray = np.r_[camera.normalized([[u, v]])[0], 1.]
            ray /= np.linalg.norm(ray)
        if (not np.isfinite(corrected).all() or not 0 <= corrected[0] < camera.width
                or not 0 <= corrected[1] < camera.height):
            continue
        ray_center = camera.normalized([corrected])[0]
        if angular_radius is None:
            boundary = np.column_stack((normalized, np.ones(len(normalized))))
            boundary /= np.linalg.norm(boundary, axis=1)[:, None]
            axis = np.r_[ray_center, 1.]
            axis /= np.linalg.norm(axis)
            angular_radius = float(np.median(np.arccos(np.clip(boundary @ axis, -1, 1))))
        candidates.append({
            'uv': corrected.tolist(), 'contour_uv': [float(u), float(v)],
            'radius': float(radius), 'area': float(area), 'bbox': list(map(int, cv2.boundingRect(hull))),
            'contour': hull[:, 0].tolist(), 'confidence': float(min(1., circularity) * solidity),
            'ray': [float(ray_center[0]), float(ray_center[1]), 1.],
            'angular_radius': angular_radius, 'geometry_valid': True,
        })
    return candidates


def detect_ball(image, camera, settings):
    candidates = find_balls(image, camera, settings)
    return candidates[0] if len(candidates) == 1 else None, len(candidates)


class CameraTracker:
    def __init__(self):
        self.reference = None
        self.manual = False

    def reset(self):
        self.reference = None
        self.manual = False

    def select(self, uv, candidates):
        if not candidates:
            raise ValueError('画像の球候補がありません。先に色を選んでください。')
        point = np.asarray(uv, float)
        candidates = [candidate for candidate in candidates
                      if np.linalg.norm(np.asarray(candidate['contour_uv']) - point) <= candidate['radius'] * 1.2]
        if not candidates:
            raise ValueError('黄色い候補円の内側をクリックしてください。')
        chosen = min(candidates, key=lambda candidate: np.linalg.norm(np.asarray(candidate['contour_uv']) - point))
        self.reference = chosen
        self.manual = True

    def detect(self, image, camera, settings):
        candidates = find_balls(image, camera, settings)
        chosen = None
        if not self.manual and len(candidates) == 1:
            chosen = candidates[0]
        elif not self.manual and len(candidates) > 1 and settings['image_dominance_ratio'] > 0:
            ranked = sorted(candidates, key=lambda candidate: candidate['area'], reverse=True)
            if ranked[0]['area'] >= ranked[1]['area'] * settings['image_dominance_ratio']:
                chosen = ranked[0]
        if chosen is None and self.reference is not None:
            matches = []
            for candidate in candidates:
                ratio = candidate['radius'] / self.reference['radius']
                distance = np.linalg.norm(np.asarray(candidate['contour_uv']) - self.reference['contour_uv'])
                if .45 <= ratio <= 2.2 and distance <= max(40., self.reference['radius'] * 3):
                    score = distance / max(self.reference['radius'], 1.) + abs(math.log(ratio))
                    matches.append((score, candidate))
            matches.sort(key=lambda item: item[0])
            if matches and (len(matches) == 1 or matches[1][0] - matches[0][0] > .3):
                chosen = matches[0][1]
        if chosen is not None:
            self.reference = chosen
        # Never return an earlier detection while the target is absent.
        return chosen, candidates


def annotate_candidates(image, candidates, selected):
    for index, candidate in enumerate(candidates):
        if candidate is selected:
            continue
        center = tuple(np.rint(candidate['contour_uv']).astype(int))
        cv2.circle(image, center, round(candidate['radius']), (0, 210, 255), 1)
        cv2.putText(image, str(index + 1), center, cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 210, 255), 1)
    return annotate(image, selected)


def annotate(image, detection):
    if detection is None:
        return image
    cv2.polylines(image, [np.asarray(detection['contour'], np.int32)], True, (0, 255, 0), 2)
    cv2.circle(image, tuple(np.rint(detection['uv']).astype(int)), 4, (0, 0, 255), -1)
    x, y, width, height = detection['bbox']
    cv2.rectangle(image, (x, y), (x + width, y + height), (0, 180, 0), 1)
    cv2.putText(image, f"u={detection['uv'][0]:.1f} v={detection['uv'][1]:.1f} "
                f"confidence={detection['confidence']:.2f}", (8, 25),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
    return image
