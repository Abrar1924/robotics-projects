"""Loosely coupled 2-D GNSS/INS Kalman filter.

State: [p_east, p_north, v_east, v_north, accel_bias_east, accel_bias_north]
The accelerometer drives the prediction; RTK positions correct it. Because the model is
linear, this is an ordinary Kalman filter, and its innovations are exactly what the
integrity tests inspect. The bias state also absorbs slowly varying tilt (gravity-leak)
errors, so its random walk is tuned to them; process noise was tuned until the NIS on
clean flights averages ~2 (its theoretical value), which makes chi-square thresholds valid.
"""
from __future__ import annotations

import math

import numpy as np


class GnssInsKF:
    def __init__(self, p0, v0, accel_noise=0.03, bias_walk=0.004, bias_sigma=0.03,
                 pos_sigma=0.02, vel_sigma=0.05, scale_sigma=0.01):
        self.x = np.r_[p0, v0, 0.0, 0.0].astype(float)
        self.P = np.diag([pos_sigma**2] * 2 + [vel_sigma**2] * 2 + [bias_sigma**2] * 2)
        self.accel_noise, self.bias_walk, self.scale_sigma = accel_noise, bias_walk, scale_sigma
        self.H = np.hstack([np.eye(2), np.zeros((2, 4))])

    def predict(self, accel_meas, dt):
        acc = accel_meas - self.x[4:6]
        self.x[0:2] += self.x[2:4] * dt + 0.5 * acc * dt * dt
        self.x[2:4] += acc * dt
        F = np.eye(6)
        F[0, 2] = F[1, 3] = dt
        F[0, 4] = F[1, 5] = -0.5 * dt * dt
        F[2, 4] = F[3, 5] = -dt
        G = np.vstack([0.5 * dt * dt * np.eye(2), dt * np.eye(2), np.zeros((2, 2))])
        # scale-factor uncertainty: acceleration error grows with the manoeuvre (turns)
        q = self.accel_noise**2 + (self.scale_sigma * float(np.hypot(*accel_meas))) ** 2
        Q = q * G @ G.T
        Q[4, 4] = Q[5, 5] = self.bias_walk**2 * dt
        self.P = F @ self.P @ F.T + Q

    def innovation(self, z, sigma):
        """Innovation, its covariance, and the normalised innovation squared (NIS)."""
        nu = np.asarray(z, dtype=float) - self.x[0:2]
        S = self.P[0:2, 0:2] + sigma * sigma * np.eye(2)
        return nu, S, float(nu @ np.linalg.solve(S, nu))

    def floor_position(self, sigma_floor):
        """Degraded fixes have correlated errors, so averaging many of them cannot make us
        more certain than this: keep the position covariance at or above sigma_floor^2."""
        lo = float(np.linalg.eigvalsh(self.P[0:2, 0:2])[0])
        if lo < sigma_floor**2:
            self.P[0:2, 0:2] += (sigma_floor**2 - lo) * np.eye(2)

    def update(self, z, sigma):
        nu, S, _ = self.innovation(z, sigma)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        I_KH = np.eye(6) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ (sigma * sigma * np.eye(2)) @ K.T

    def constrain_lateral(self, travel_direction, sigma):
        """Pseudo-measurement: the velocity across the direction of travel is zero, within `sigma`.

        Used while GNSS positions are distrusted. A vehicle that flies nose-first (or a rover that
        does not slip sideways) moves along a direction the heading sensors still know, so the
        filter can keep its cross-track velocity honest without any position fix.
        """
        h = np.zeros(6)
        h[2], h[3] = -math.sin(travel_direction), math.cos(travel_direction)
        s_var = float(h @ self.P @ h) + sigma * sigma
        K = self.P @ h / s_var
        self.x = self.x - K * float(h @ self.x)
        I_KH = np.eye(6) - np.outer(K, h)
        self.P = I_KH @ self.P @ I_KH.T + sigma * sigma * np.outer(K, K)
