"""Sensor models: MEMS accelerometer and RTK-GNSS receiver."""
from __future__ import annotations

import math

import numpy as np


class Accelerometer:
    """Horizontal acceleration in the local frame at 50 Hz, with consumer-MEMS error sources.

    - turn-on bias (~2 mg) with a slow random walk
    - tilt error: a small attitude-estimate error leaks gravity into the horizontal axes
      (0.1 deg of tilt ~ 0.017 m/s^2), varying slowly as the attitude filter wanders
    - scale-factor error (~0.5 %) on manoeuvring accelerations
    - white noise
    Attitude itself is not simulated; the tilt term stands in for its error.
    """

    def __init__(self, rng, noise=0.02, bias_sigma=0.02, bias_walk=2e-4, tilt_sigma=0.015,
                 tilt_tau=20.0, scale_sigma=0.005):
        self.rng, self.noise, self.bias_walk = rng, noise, bias_walk
        self.tilt_sigma, self.tilt_tau = tilt_sigma, tilt_tau
        self.bias = rng.normal(0.0, bias_sigma, 2)
        self.tilt = rng.normal(0.0, tilt_sigma, 2)
        self.scale = 1.0 + rng.normal(0.0, scale_sigma, 2)

    def sample(self, accel_true, dt):
        self.bias = self.bias + self.bias_walk * math.sqrt(dt) * self.rng.normal(size=2)
        a = math.exp(-dt / self.tilt_tau)
        self.tilt = a * self.tilt + math.sqrt(1 - a * a) * self.tilt_sigma * self.rng.normal(size=2)
        return self.scale * accel_true + self.bias + self.tilt + self.noise * self.rng.normal(size=2)


class RTKReceiver:
    """RTK-fixed position at 10 Hz with 2 cm noise and an honest reported accuracy."""

    def __init__(self, rng, sigma=0.02):
        self.rng, self.sigma = rng, sigma

    def measure(self, p_true):
        return p_true + self.rng.normal(0.0, self.sigma, 2), self.sigma
