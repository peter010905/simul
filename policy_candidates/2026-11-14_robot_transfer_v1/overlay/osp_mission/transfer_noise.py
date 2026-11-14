"""Synthetic sensor stressors, not measurements of a particular real robot.

Simulator-side only. No map, marker, world pose or evaluator inputs are used.
Invalid/missing returns remain invalid, never fabricated clear space.
"""
import copy
import math
import numpy as np

PROFILES = {
    'none': dict(std=0., bias=0., ray_drop=0., frame_drop=0., delay=0., linear=1., angular=1.),
    'mild': dict(std=.01, bias=.005, ray_drop=.01, frame_drop=.02, delay=.06, linear=.98, angular=1.01),
    'moderate': dict(std=.02, bias=-.01, ray_drop=.03, frame_drop=.05, delay=.12, linear=1.03, angular=.98),
}


class SensorStress:
    def __init__(self, profile='none', seed=0):
        if profile not in PROFILES:
            raise ValueError('Unknown transfer profile')
        self.name = profile
        self.profile = PROFILES[profile]
        self.rng = np.random.default_rng(seed)
        self.received = 0
        self.dropped = 0

    def scan(self, message):
        self.received += 1
        p = self.profile
        if self.name == 'none':
            return message
        if self.rng.random() < p['frame_drop']:
            self.dropped += 1
            return None
        result = copy.deepcopy(message)
        values = np.array(message.ranges, dtype=float)
        valid = np.isfinite(values) & (values > message.range_min) & (values < message.range_max)
        values[valid] += self.rng.normal(p['bias'], p['std'], int(valid.sum()))
        dropped = valid & (self.rng.random(values.size) < p['ray_drop'])
        outside = valid & ((values <= message.range_min) | (values >= message.range_max))
        values[dropped | outside] = np.nan
        result.ranges = values.tolist()
        # Keep the acquisition stamp; transport delay must be visible to TF and
        # sensor freshness checks. Restamping would hide the impairment.
        return result

    def twist(self, message):
        result = copy.deepcopy(message)
        factors = [self.profile['linear'], 1., 1., 1., 1., self.profile['angular']]
        result.twist.twist.linear.x *= factors[0]
        result.twist.twist.angular.z *= factors[5]
        covariance = np.asarray(message.twist.covariance).reshape(6, 6)
        result.twist.covariance = (covariance * np.outer(factors, factors)).ravel().tolist()
        return result


def environment_profile():
    import os
    profile = os.environ.get('OSP_TRANSFER_PROFILE', 'none')
    seed = int(os.environ.get('OSP_TRANSFER_SEED', '0'))
    if profile not in PROFILES:
        raise ValueError('Unknown OSP_TRANSFER_PROFILE')
    return profile, seed
