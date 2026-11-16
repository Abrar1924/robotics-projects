"""Statistical RTK-GNSS receiver model.

Fix quality follows the sky view at the antenna and uses NMEA GGA codes:
4 = RTK fixed (~2 cm), 5 = RTK float (decimetres, with a slowly varying bias),
0 = no solution. After any outage the receiver needs several seconds of open sky
to re-resolve integer ambiguities before it reports fixed again.
"""
from __future__ import annotations

import math

import numpy as np
from shapely.geometry import Point
from shapely.ops import nearest_points
from shapely.prepared import prep

from .world import CANOPY_DEEP, CANOPY_LIGHT, MULTIPATH, OPEN, SEVERITY

FIXED, FLOAT, NONE = 4, 5, 0
QUALITY_NAME = {FIXED: "RTK fixed", FLOAT: "RTK float", NONE: "no fix"}


class RTKReceiver:
    def __init__(self, yard, rng, rate_hz=5.0, fixed_noise=0.015, float_noise=0.10,
                 float_bias=0.30, multipath_bias=0.35, multipath_push=0.25,
                 outlier_prob=0.03, fixed_window_rate=0.12, convergence=(4.0, 10.0),
                 bias_tau=20.0):
        self.rng = rng
        self.period = 1.0 / rate_hz
        self._zones = [(z.kind, prep(z.geometry)) for z in yard.zones]
        self._walls = yard.keep_out.get("house")
        self.fixed_noise, self.float_noise = fixed_noise, float_noise
        self.float_bias, self.multipath_bias = float_bias, multipath_bias
        self.multipath_push, self.outlier_prob = multipath_push, outlier_prob
        self.fixed_window_rate, self.convergence = fixed_window_rate, convergence
        self.bias_tau = bias_tau
        self.bias = np.zeros(2)
        self.needs_convergence = False
        self.converged_at = 0.0
        self.window_until = -1.0

    def sky_view(self, x, y):
        p, worst = Point(x, y), OPEN
        for kind, zone in self._zones:
            if SEVERITY[kind] > SEVERITY[worst] and zone.contains(p):
                worst = kind
        return worst

    def measure(self, t, x, y):
        """Return (quality, measured_xy or None, reported 1-sigma in metres)."""
        env = self.sky_view(x, y)
        # Float error is correlated in time (first-order Gauss-Markov), not white noise.
        target = {OPEN: 0.0, MULTIPATH: self.multipath_bias,
                  CANOPY_LIGHT: self.float_bias, CANOPY_DEEP: self.float_bias}[env]
        a = math.exp(-self.period / self.bias_tau)
        self.bias = a * self.bias + math.sqrt(1.0 - a * a) * target * self.rng.normal(size=2)

        if env == CANOPY_DEEP:
            quality = NONE
        elif env == CANOPY_LIGHT:
            if t >= self.window_until and self.rng.random() < self.fixed_window_rate * self.period:
                self.window_until = t + self.rng.uniform(2.0, 4.0)
            quality = FIXED if t < self.window_until else FLOAT
        elif env == MULTIPATH:
            quality = FLOAT
        else:
            if self.needs_convergence:
                self.needs_convergence = False
                self.converged_at = t + self.rng.uniform(*self.convergence)
            quality = FIXED if t >= self.converged_at else FLOAT
        if env != OPEN:
            self.needs_convergence = quality != FIXED

        if quality == NONE:
            return NONE, None, None
        truth = np.array([x, y], dtype=float)
        if quality == FIXED:
            return FIXED, truth + self.rng.normal(0.0, self.fixed_noise, 2), 0.02
        z = truth + self.bias + self.rng.normal(0.0, self.float_noise, 2)
        if env == MULTIPATH and self._walls is not None:
            # Reflections lengthen the signal path: the solution is pushed away from the wall.
            wall = nearest_points(self._walls, Point(x, y))[0]
            away = truth - np.array([wall.x, wall.y])
            z = z + self.multipath_push * away / max(np.linalg.norm(away), 1e-6)
            if self.rng.random() < self.outlier_prob:
                z = z + self.rng.normal(0.0, 1.5, 2)  # occasional multipath jump
        return FLOAT, z, 0.25
