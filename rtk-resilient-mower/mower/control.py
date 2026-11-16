"""Pure-pursuit path tracking with turn-in-place for sharp corners."""
from __future__ import annotations

import math

import numpy as np

from .robot import wrap


class PurePursuit:
    def __init__(self, path, lookahead=0.4, w_max=1.2, rotate_above=0.7, window=3.0, spacing=0.1):
        self.lookahead, self.w_max, self.rotate_above = lookahead, w_max, rotate_above
        self.window_pts = max(5, int(window / spacing))
        self.set_path(path)

    def set_path(self, path):
        self.path = np.asarray(path, dtype=float)
        seg = np.diff(self.path, axis=0)
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
        self.idx = 0
        self.done = len(self.path) < 2

    def command(self, x, y, th, v_limit):
        """Return (v, w) to follow the path from the estimated pose (x, y, th)."""
        if self.done:
            return 0.0, 0.0
        # Only search a short window ahead so the tracker never jumps to a neighbouring swath.
        end = min(len(self.path), self.idx + self.window_pts)
        d = np.hypot(self.path[self.idx:end, 0] - x, self.path[self.idx:end, 1] - y)
        self.idx += int(np.argmin(d))
        last = len(self.path) - 1
        if self.idx >= last - 1 and math.hypot(self.path[last, 0] - x, self.path[last, 1] - y) < 0.15:
            self.idx, self.done = last, True
            return 0.0, 0.0
        if v_limit <= 0.0:
            return 0.0, 0.0
        j = min(int(np.searchsorted(self.s, self.s[self.idx] + self.lookahead)), last)
        dx, dy = self.path[j, 0] - x, self.path[j, 1] - y
        alpha = wrap(math.atan2(dy, dx) - th)
        if abs(alpha) > self.rotate_above:
            return 0.0, math.copysign(min(self.w_max, 2.0 * abs(alpha)), alpha)
        kappa = 2.0 * math.sin(alpha) / max(math.hypot(dx, dy), 1e-3)
        v = v_limit / (1.0 + 1.5 * abs(kappa))
        return v, float(np.clip(v * kappa, -self.w_max, self.w_max))

    def cross_track(self, x, y):
        """Distance from (x, y) to the path near the current index."""
        lo, hi = max(0, self.idx - 10), min(len(self.path) - 1, self.idx + 10)
        p = self.path[lo:hi + 1]
        if len(p) < 2:
            return float(math.hypot(p[0, 0] - x, p[0, 1] - y))
        a, ab = p[:-1], np.diff(p, axis=0)
        t = ((x - a[:, 0]) * ab[:, 0] + (y - a[:, 1]) * ab[:, 1]) / np.maximum((ab**2).sum(1), 1e-12)
        t = np.clip(t, 0.0, 1.0)
        return float(np.min(np.hypot(a[:, 0] + t * ab[:, 0] - x, a[:, 1] + t * ab[:, 1] - y)))
