from collections import deque
import copy
import time

import numpy as np


class SampleCollector:
    def __init__(self, settings):
        self.settings = settings
        self.samples = []
        self.history = deque(maxlen=60)
        self.latest = None
        self.last_capture = float('-inf')
        self.next_id = 1
        self.auto = False

    def observe(self, observation):
        now = time.monotonic()
        gap_limit = min(self.settings['stale_timeout'], 2.5 / self.settings['processing_hz'])
        if self.history and now - self.history[-1][0] > gap_limit:
            self.history.clear()
        self.latest = observation
        if observation is None:
            return
        self.history.append((now, observation))
        cutoff = now - max(self.settings['stationary_duration'], 3 / self.settings['processing_hz']) - .15
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

    def stability(self):
        report = {'ready': False, 'count': len(self.history), 'duration': 0.,
                  'position_spread': None, 'pixel_spread': None, 'reason': '',
                  'required_duration': self.settings['stationary_duration'],
                  'position_limit': self.settings['stationary_position'],
                  'pixel_limit': self.settings['stationary_pixels']}
        if self.latest is None or not self.history:
            report['reason'] = '同期したカメラ・LiDARの対象待ち'
            return report
        if time.monotonic() - self.history[-1][0] > self.settings['stale_timeout']:
            report['reason'] = '対象データの鮮度切れ'
            return report
        report['duration'] = self.history[-1][0] - self.history[0][0]
        if len(self.history) < 3 or report['duration'] + .001 < self.settings['stationary_duration']:
            report['reason'] = '静止確認中'
            return report
        xyz = np.array([item['xyz'] for _, item in self.history])
        uv = np.array([item['uv'] for _, item in self.history])
        center = np.median(xyz, axis=0)
        pixel_center = np.median(uv, axis=0)
        position_deviation = np.linalg.norm(xyz - center, axis=1)
        pixel_deviation = np.linalg.norm(uv - pixel_center, axis=1)
        report['position_spread'] = float(np.quantile(position_deviation, .8))
        report['pixel_spread'] = float(np.quantile(pixel_deviation, .8))
        # A current target at a new position must never be accepted merely
        # because most earlier observations were stationary.
        current_position = float(np.linalg.norm(np.array(self.latest['xyz']) - center))
        current_pixel = float(np.linalg.norm(np.array(self.latest['uv']) - pixel_center))
        drift = float(np.linalg.norm(np.median(xyz[-2:], axis=0) - np.median(xyz[:2], axis=0)))
        pixel_drift = float(np.linalg.norm(np.median(uv[-2:], axis=0) - np.median(uv[:2], axis=0)))
        if max(report['position_spread'], current_position, drift) > self.settings['stationary_position']:
            report['reason'] = 'LiDARの揺れ幅が許容値超過'
        elif max(report['pixel_spread'], current_pixel, pixel_drift) > self.settings['stationary_pixels']:
            report['reason'] = '画像の揺れ幅が許容値超過'
        else:
            report['ready'] = True
            report['reason'] = '取得可能'
        return report

    def stationary(self):
        return self.stability()['ready']

    def capture(self, role='auto'):
        if self.latest is None or not self.history:
            raise ValueError('同期したカメラ・LiDARの認識が必要です。')
        if time.monotonic() - self.history[-1][0] > self.settings['stale_timeout']:
            raise ValueError('センサデータが古くなっています。')
        if not self.stationary():
            raise ValueError('ボールが静止するまで待ってください。')
        if time.monotonic() - self.last_capture < self.settings['sample_interval']:
            raise ValueError('サンプル取得間隔を待ってください。')
        if len(self.samples) >= self.settings['max_samples']:
            raise ValueError('サンプル数の上限に達しました。')
        observations = [item for received, item in self.history
                        if time.monotonic() - received <= self.settings['stale_timeout']]
        xyz = np.array([item['xyz'] for item in observations])
        uv = np.array([item['uv'] for item in observations])
        score = (np.linalg.norm(xyz - np.median(xyz, axis=0), axis=1) / self.settings['stationary_position']
                 + np.linalg.norm(uv - np.median(uv, axis=0), axis=1) / self.settings['stationary_pixels'])
        representative = observations[int(np.argmin(score))]
        positions = np.array([sample['xyz'] for sample in self.samples])
        if len(positions) and np.min(np.linalg.norm(positions - representative['xyz'], axis=1)) < self.settings['duplicate_distance']:
            raise ValueError('既存サンプルと同じ位置です。ボールを移動してください。')
        if role == 'auto':
            role = 'validation' if (len(self.samples) + 1) % self.settings['validation_every'] == 0 else 'train'
        if role not in ('train', 'validation'):
            raise ValueError('サンプルの用途が無効です。')
        sample = copy.deepcopy(representative)
        sample['stability'] = self.stability()
        sample.update(id=self.next_id, role=role)
        self.samples.append(sample)
        self.next_id += 1
        self.last_capture = time.monotonic()
        return sample

    def delete(self, ids):
        selected = set(ids)
        self.samples = [sample for sample in self.samples if sample['id'] not in selected]
