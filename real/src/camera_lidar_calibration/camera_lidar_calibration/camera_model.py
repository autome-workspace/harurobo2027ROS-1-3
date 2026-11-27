import cv2
import numpy as np


class CameraModel:
    def __init__(self, width, height, matrix, distortion, model='plumb_bob',
                 rectified=False, rectification=None, projection=None):
        self.width, self.height = int(width), int(height)
        self.matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
        self.distortion = np.asarray(distortion, dtype=float).ravel()
        self.model = model
        self.rectified = bool(rectified)
        self.rectification = np.eye(3) if rectification is None else np.asarray(rectification, float).reshape(3, 3)
        self.projection = (np.column_stack((self.matrix, np.zeros(3))) if projection is None
                           else np.asarray(projection, float).reshape(3, 4))
        if (min(self.width, self.height) <= 0 or min(self.matrix[0, 0], self.matrix[1, 1]) <= 0
                or not all(np.isfinite(a).all() for a in
                           (self.matrix, self.distortion, self.rectification, self.projection))):
            raise ValueError('カメラ内部較正が無効です。CameraInfoまたはYAMLを確認してください。')
        if not np.allclose(self.matrix[2], [0, 0, 1]) or abs(self.matrix[0, 1]) > 1e-8:
            raise ValueError('未対応の内部行列です。ゼロスキューの標準カメラ行列を指定してください。')
        if model not in ('plumb_bob', 'rational_polynomial', 'equidistant', ''):
            raise ValueError('未対応のレンズ歪みモデルです。')
        if model == 'equidistant' and self.distortion.size != 4:
            raise ValueError('魚眼歪み係数は4要素が必要です。')
        if model != 'equidistant' and self.distortion.size not in (0, 4, 5, 8, 12, 14):
            raise ValueError('歪み係数の要素数が無効です。')
        if self.rectified:
            if (np.linalg.norm(self.projection[:, 3]) > 1e-9
                    or min(self.projection[0, 0], self.projection[1, 1]) <= 0
                    or not np.allclose(self.rectification.T @ self.rectification, np.eye(3), atol=1e-5)
                    or not np.isclose(np.linalg.det(self.rectification), 1)
                    or not np.allclose(self.projection[2], [0, 0, 1, 0])):
                raise ValueError('単眼補正画像のCameraInfo.R/Pを確認してください。ステレオの基線項には未対応です。')

    @classmethod
    def from_info(cls, msg, rectified=False):
        return cls(msg.width, msg.height, msg.k, msg.d, msg.distortion_model,
                   rectified, msg.r, msg.p)

    @classmethod
    def from_dict(cls, data):
        return cls(data['width'], data['height'], data['matrix'], data['distortion'], data['model'],
                   data.get('rectified', False), data.get('rectification'), data.get('projection'))

    @classmethod
    def from_yaml(cls, path, rectified=False):
        from pathlib import Path
        import yaml
        data = yaml.safe_load(Path(path).expanduser().read_text())
        return cls(data['image_width'], data['image_height'], data['camera_matrix']['data'],
                   data['distortion_coefficients']['data'], data['distortion_model'], rectified,
                   data['rectification_matrix']['data'], data['projection_matrix']['data'])

    def as_dict(self):
        return {'width': self.width, 'height': self.height, 'matrix': self.matrix.tolist(),
                'distortion': self.distortion.tolist(), 'model': self.model, 'rectified': self.rectified,
                'rectification': self.rectification.tolist(), 'projection': self.projection.tolist()}

    def normalized(self, pixels):
        points = np.asarray(pixels, float).reshape(-1, 1, 2)
        if self.rectified:
            rays = np.column_stack(((points[:, 0, 0] - self.projection[0, 2]) / self.projection[0, 0],
                                    (points[:, 0, 1] - self.projection[1, 2]) / self.projection[1, 1],
                                    np.ones(len(points))))
            rays = rays @ self.rectification
            return rays[:, :2] / rays[:, 2:3]
        if self.model == 'equidistant':
            return cv2.fisheye.undistortPoints(points, self.matrix, self.distortion)[:, 0]
        return cv2.undistortPoints(points, self.matrix, self.distortion)[:, 0]

    def project(self, camera_points):
        points = np.asarray(camera_points, float).reshape(-1, 3)
        if not len(points):
            return np.empty((0, 2))
        if self.rectified:
            transformed = points @ self.rectification.T
            return (transformed @ self.projection[:, :3].T)[:, :2] / transformed[:, 2:3]
        if self.model == 'equidistant':
            return cv2.fisheye.projectPoints(points.reshape(-1, 1, 3), np.zeros(3), np.zeros(3),
                                            self.matrix, self.distortion)[0][:, 0]
        return cv2.projectPoints(points, np.zeros(3), np.zeros(3), self.matrix, self.distortion)[0][:, 0]
