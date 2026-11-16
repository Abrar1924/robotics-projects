"""Extended Kalman filter fusing wheel odometry, a yaw gyro and RTK-GNSS positions.

State: [x, y, heading, gyro_bias, odom_scale, float_bias_x, float_bias_y]

- Predict with the encoder speed (times a learned scale) and the bias-corrected gyro.
- Update with RTK positions. Fixed solutions are near-truth; float solutions carry a
  slowly varying bias, modelled as two Gauss-Markov states so a float fix cannot
  make the filter overconfident.
- A chi-square gate rejects multipath outliers. If GNSS keeps disagreeing *consistently*
  (the same offset fix after fix), the estimate has drifted: the position covariance is
  reopened instead of rejecting good data forever. Random jumps never trigger this.
- The odometry-scale state learns grass slip while RTK is fixed, which keeps dead
  reckoning usable under canopy.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from .robot import wrap

CHI2_2DOF_999 = 13.82


class MowerEKF:
    def __init__(self, pose, pos_sigma=0.02, heading_sigma=0.03, slip_noise=0.45,
                 gyro_noise=0.005, bias_walk=1e-4, scale_walk=2e-3, float_bias_sigma=0.30,
                 float_bias_tau=20.0, gate=CHI2_2DOF_999, max_rejects=10):
        self.x = np.array([pose[0], pose[1], pose[2], 0.0, 1.0, 0.0, 0.0], dtype=float)
        self.P = np.diag([pos_sigma**2, pos_sigma**2, heading_sigma**2, 0.005**2, 0.05**2,
                          float_bias_sigma**2, float_bias_sigma**2])
        # slip_noise is the white-noise equivalent of ~3 % slip correlated over ~6 s at 20 Hz.
        self.slip_noise, self.gyro_noise = slip_noise, gyro_noise
        self.bias_walk, self.scale_walk = bias_walk, scale_walk
        self.fb_sigma, self.fb_tau = float_bias_sigma, float_bias_tau
        self.gate, self.max_rejects = gate, max_rejects
        self.rejected = deque(maxlen=max_rejects)   # recent consecutive gated-out innovations
        self.last_nis = 0.0

    @property
    def pose(self):
        return self.x[:3]

    @property
    def sigma_pos(self):
        """Worst-direction 1-sigma horizontal position uncertainty (m)."""
        return float(math.sqrt(max(np.linalg.eigvalsh(self.P[:2, :2])[-1], 0.0)))

    def predict(self, v_wheel, w_gyro, dt):
        x, y, th, bg, k, bx, by = self.x
        w = w_gyro - bg
        v = k * v_wheel
        mid = th + 0.5 * w * dt
        c, s = math.cos(mid), math.sin(mid)
        a = math.exp(-dt / self.fb_tau)
        self.x = np.array([x + v * c * dt, y + v * s * dt, wrap(th + w * dt), bg, k, a * bx, a * by])

        F = np.eye(7)
        F[0, 2], F[1, 2] = -v * s * dt, v * c * dt
        F[0, 3], F[1, 3] = 0.5 * v * s * dt * dt, -0.5 * v * c * dt * dt
        F[0, 4], F[1, 4] = v_wheel * c * dt, v_wheel * s * dt
        F[2, 3] = -dt
        F[5, 5] = F[6, 6] = a
        sv = self.slip_noise * abs(v_wheel) * dt
        Q = np.zeros((7, 7))
        Q[0, 0] = Q[1, 1] = sv * sv + (1e-4 * dt) ** 2
        Q[2, 2] = (self.gyro_noise * dt) ** 2
        Q[3, 3] = self.bias_walk**2 * dt
        Q[4, 4] = self.scale_walk**2 * dt
        Q[5, 5] = Q[6, 6] = self.fb_sigma**2 * (1.0 - a * a)
        self.P = F @ self.P @ F.T + Q

    def update_position(self, z, sigma, float_solution=False):
        """Fuse one RTK position. Returns False if the fix was gated out as an outlier."""
        H = np.zeros((2, 7))
        H[0, 0] = H[1, 1] = 1.0
        if float_solution:
            H[0, 5] = H[1, 6] = 1.0
        R = np.eye(2) * sigma * sigma
        innov = np.asarray(z, dtype=float) - H @ self.x
        S = H @ self.P @ H.T + R
        nis = float(innov @ np.linalg.solve(S, innov))
        self.last_nis = nis
        if nis > self.gate:
            self.rejected.append(innov)
            if len(self.rejected) < self.max_rejects:
                return False
            recent = np.array(self.rejected)
            mean = recent.mean(axis=0)
            spread = float(np.sqrt(((recent - mean) ** 2).sum(axis=1).mean()))
            if spread > 0.5 * np.linalg.norm(mean) + 3.0 * sigma:
                return False          # scattered disagreement: outliers, keep rejecting
            if np.linalg.norm(innov - mean) > 3.0 * (spread + sigma):
                return False          # this fix is not part of the consistent offset
            # Consistent offset fix after fix: our estimate drifted. Reopen and accept.
            self.P[:2, :2] += np.outer(mean, mean) + np.eye(2) * spread**2
            S = H @ self.P @ H.T + R
        self.rejected.clear()
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innov
        self.x[2] = wrap(self.x[2])
        I_KH = np.eye(7) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        return True
