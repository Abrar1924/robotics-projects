"""Differential-drive mower on turf, with wheel encoders and a yaw-rate gyro."""
from __future__ import annotations

import math

import numpy as np


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DiffDriveMower:
    """Unicycle model with grass effects the encoders cannot see.

    - along-track slip (~4 % mean, varies over time): wheels turn more than the robot moves
    - side-slip on uneven turf: sideways drift neither encoders nor gyro measure
    - small yaw disturbances (real rotation, so the gyro does sense them)
    """

    def __init__(self, pose, rng, accel=0.8, yaw_accel=3.0, slip_mean=0.04, slip_sigma=0.03,
                 side_slip_sigma=0.03, slip_tau=6.0, yaw_disturbance=0.02,
                 gyro_noise=0.005, gyro_bias_walk=1e-4):
        self.x, self.y, self.th = (float(v) for v in pose)
        self.rng = rng
        self.v = self.w = 0.0
        self.accel, self.yaw_accel = accel, yaw_accel
        self.slip_mean, self.slip_sigma, self.slip_tau = slip_mean, slip_sigma, slip_tau
        self.side_slip_sigma, self.yaw_disturbance = side_slip_sigma, yaw_disturbance
        self.slip, self.side_slip, self.yaw_dist = slip_mean, 0.0, 0.0
        self.gyro_noise, self.gyro_bias_walk = gyro_noise, gyro_bias_walk
        self.gyro_bias = rng.normal(0.0, 0.003)
        self.w_ground = 0.0

    def _gauss_markov(self, value, mean, sigma, dt):
        a = math.exp(-dt / self.slip_tau)
        return a * value + (1.0 - a) * mean + math.sqrt(1.0 - a * a) * sigma * self.rng.normal()

    def step(self, v_cmd, w_cmd, dt):
        self.v += float(np.clip(v_cmd - self.v, -self.accel * dt, self.accel * dt))
        self.w += float(np.clip(w_cmd - self.w, -self.yaw_accel * dt, self.yaw_accel * dt))
        self.slip = self._gauss_markov(self.slip, self.slip_mean, self.slip_sigma, dt)
        self.side_slip = self._gauss_markov(self.side_slip, 0.0, self.side_slip_sigma, dt)
        self.yaw_dist = self._gauss_markov(self.yaw_dist, 0.0, self.yaw_disturbance, dt)
        moving = abs(self.v) > 0.01
        along = self.v * (1.0 - min(max(self.slip + 0.05 * abs(self.w), 0.0), 0.3))
        lateral = self.side_slip * self.v
        self.w_ground = self.w + (self.yaw_dist if moving else 0.0)
        mid = self.th + 0.5 * self.w_ground * dt
        c, s = math.cos(mid), math.sin(mid)
        self.x += (along * c - lateral * s) * dt
        self.y += (along * s + lateral * c) * dt
        self.th = wrap(self.th + self.w_ground * dt)
        self.gyro_bias += self.gyro_bias_walk * math.sqrt(dt) * self.rng.normal()

    def wheel_speed(self):
        """Encoder speed: measures wheel rotation, so slip is invisible to it."""
        return self.v * (1.0 + 0.01 * self.rng.normal())

    def gyro(self):
        return self.w_ground + self.gyro_bias + self.gyro_noise * self.rng.normal()
