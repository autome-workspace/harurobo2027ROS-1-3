"""Validate calibration geometry and checkerboard detection with synthetic images."""
from pathlib import Path
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from core.camera_calibration import Calibration

calibration = Calibration(9, 6, 25.)
# Ten by seven squares contain nine by six internal corners.
image = np.full((560, 800, 3), 200, dtype=np.uint8)
for row in range(7):
    for column in range(10):
        image[70 + row * 60:70 + (row + 1) * 60,
              100 + column * 60:100 + (column + 1) * 60] = 255 if (row + column) % 2 else 0
assert calibration.detect(image)
calibration.capture()
try:
    calibration.capture()
except ValueError:
    pass
else:
    raise AssertionError('duplicate board accepted')

calibration = Calibration(9, 6, 25.)
calibration.size = (640, 480)
matrix = np.array([[560., 0., 320.], [0., 565., 240.], [0., 0., 1.]])
points = np.zeros((54, 3), np.float32)
points[:, :2] = np.mgrid[:9, :6].T.reshape(-1, 2) * .025
for index in range(16):
    rotation = np.array([.25 * np.sin(index), .3 * np.cos(index), .05 * index])
    translation = np.array([-.10 + .02 * np.sin(index), -.065, .50 + .015 * index])
    corners, _ = cv2.projectPoints(points, rotation, translation, matrix, np.zeros(5))
    calibration.corners = corners.astype(np.float32)
    calibration.descriptor = corners.reshape(-1, 2) / np.array(calibration.size)
    calibration.stamp = time.monotonic()
    # Cover diverse poses in the numerical solver independently of capture selection.
    calibration.samples.append(calibration.corners)
rms = calibration.solve()
assert rms < .01
recovered = np.array(calibration.model['camera_matrix']['data']).reshape(3, 3)
assert np.allclose(recovered, matrix, atol=.1), recovered
assert calibration.model['image_width'] == 640
assert len(calibration.model['projection_matrix']['data']) == 12
print('PASS: checkerboard detection, duplicate rejection, numerical camera calibration')
