"""Integrity tests on filter innovations, plus a learned anomaly detector.

Classical, physics-based tests (thresholds from chi-square theory, not tuned on attacks):
  snapshot   NIS of a single fix exceeds chi2(2) at alpha            -> catches jumps
  window     sum of NIS over the last 5 s exceeds chi2(2n) at alpha  -> catches persistent offsets
             (only over fixes that claim RTK-fixed precision: float errors are correlated from fix
             to fix, which breaks the independence the test assumes. Spoofers claim fixed.)
  bias       estimated accelerometer bias exceeds what the sensor can physically have
             -> a spoofer that drags slowly forces the filter to "explain" the fake motion
                with an impossible bias
Learned:
  iforest    Isolation Forest on innovation statistics, trained on clean and honestly
             degraded flights only (no attack examples), threshold set on held-out clean flights.
             Like the window test it only scores fixes that claim RTK-fixed precision.
Dual antenna (see dual_antenna.py):
  baseline   measured antenna separation differs from the known 0.20 m   -> single-transmitter spoofing
  heading    GNSS heading from the baseline disagrees with the gyro         -> faked but frozen baselines
  course     direction of travel disagrees with where the nose points       -> a perfect baseline, dragged position
"""
from __future__ import annotations

from collections import deque

import numpy as np
from scipy.stats import chi2
from sklearn.ensemble import IsolationForest

from .dual_antenna import PERSIST, THRESHOLD as DUAL_THRESHOLD, persistent

ALPHA = 1e-5          # per-test false-alarm probability per epoch
PRECISE = 0.05        # m: reported accuracy at or below this counts as an RTK-fixed claim
WINDOW = 50           # epochs (5 s at 10 Hz)
BIAS_LIMIT = 0.12     # m/s^2: 4 x the 1-sigma bias + tilt error budget, plus scale-factor margin

CLASSICAL = ("snapshot", "window", "bias")
DUAL = ("baseline", "heading", "course")
NAMES = {"snapshot": "Snapshot χ² (one fix)", "window": "Sliding-window χ² (5 s)",
         "bias": "Accelerometer-bias plausibility", "iforest": "Isolation Forest (learned)",
         "baseline": "Dual-antenna baseline length", "heading": "Dual-antenna heading vs gyro",
         "course": "Course vs heading (3 s)",
         "any": "Any classical test", "any_dual": "Any test, with dual antenna"}


def rolling_sum(x, n):
    c = np.cumsum(np.r_[0.0, x])
    out = c[n:] - c[:-n]
    return np.r_[c[1:n], out] if len(x) >= n else c[1:]


def precise_mask(log):
    """Epochs with a fix that claims RTK-fixed precision (what a spoofer always claims)."""
    return log.ep_ok & (np.nan_to_num(log.sigma, nan=1.0) <= PRECISE)


def classical_alarms(log, alpha=ALPHA, window=WINDOW, bias_limit=BIAS_LIMIT):
    """Per-epoch alarms of each classical test on a recorded (unprotected) flight."""
    ok = log.ep_ok
    nis = np.where(ok, log.nis, 0.0)
    precise = precise_mask(log)
    count = rolling_sum(precise.astype(float), window)
    total = rolling_sum(np.where(precise, nis, 0.0), window)
    win = (count >= 0.8 * window) & (total > chi2.ppf(1 - alpha, np.maximum(2 * count, 2)))
    return {"snapshot": ok & (nis > chi2.ppf(1 - alpha, 2)),
            "window": precise & win,
            "bias": np.abs(log.bias).max(axis=1) > bias_limit}


def dual_alarms(log):
    """Per-epoch alarms of the dual-antenna tests (each needs PERSIST epochs in a row)."""
    stat = np.nan_to_num(log.bl_stat, nan=0.0)
    nis = np.nan_to_num(log.h_nis, nan=0.0)
    course = np.nan_to_num(log.c_stat, nan=0.0)
    return {"baseline": persistent(stat > DUAL_THRESHOLD), "heading": persistent(nis > DUAL_THRESHOLD),
            "course": course > DUAL_THRESHOLD}   # already a 3 s average, so no extra persistence


def features(log):
    """Innovation statistics per epoch, over fixes that claim RTK-fixed precision."""
    ok = precise_mask(log)
    nis = np.where(ok, log.nis, 0.0)
    n_ok = lambda n: np.maximum(rolling_sum(ok.astype(float), n), 1.0)  # noqa: E731
    mean20 = rolling_sum(nis, 20) / n_ok(20)
    mean100 = rolling_sum(nis, 100) / n_ok(100)
    nu = np.where(ok[:, None], log.innov, 0.0)
    mean_nu = np.c_[rolling_sum(nu[:, 0], 30), rolling_sum(nu[:, 1], 30)] / n_ok(30)[:, None]
    scale = np.sqrt(rolling_sum(np.where(ok, log.s_trace / 2.0, 0.0), 30) / n_ok(30))
    lag = 100  # 10 s: how fast the bias estimate is moving
    drift = np.vstack([np.zeros((lag, 2)), log.bias[lag:] - log.bias[:-lag]])
    return np.c_[np.log(nis + 1e-6), np.log(mean20 + 1e-6), np.log(mean100 + 1e-6),
                 np.hypot(mean_nu[:, 0], mean_nu[:, 1]) / np.maximum(scale, 1e-9),
                 np.abs(log.bias).max(axis=1) / (BIAS_LIMIT / 4.0),
                 np.hypot(drift[:, 0], drift[:, 1]) / (BIAS_LIMIT / 4.0)]


class LearnedDetector:
    """Isolation Forest over innovation statistics with a persistence requirement."""

    def __init__(self, persistence=3, seed=0):
        self.persistence = persistence
        self.model = IsolationForest(n_estimators=300, random_state=seed)
        self.threshold = None

    def _scores(self, log):
        return -self.model.score_samples(features(log))   # higher = more anomalous

    def _persistent(self, log, scores):
        bad = precise_mask(log) & (scores > self.threshold)
        run, out = 0, np.zeros_like(bad)
        for i, b in enumerate(bad):
            run = run + 1 if b else 0
            out[i] = run >= self.persistence
        return out

    def fit(self, logs):
        X = np.vstack([features(lg)[precise_mask(lg)] for lg in logs])
        self.model.fit(X)
        return self

    def calibrate(self, logs, margin=1.05):
        """Threshold = the highest score held for `persistence` epochs on clean validation flights."""
        worst = 0.0
        for lg in logs:
            s = np.where(precise_mask(lg), self._scores(lg), -np.inf)
            held = np.lib.stride_tricks.sliding_window_view(s, self.persistence).min(axis=1)
            worst = max(worst, float(held.max()))
        self.threshold = worst * margin
        return self

    def scores(self, log):
        return self._scores(log)

    def alarms(self, log):
        return self._persistent(log, self._scores(log))


def first_alarm(ep_t, alarm, start):
    """Delay (s) from `start` to the first alarm at or after it; NaN if none."""
    hit = np.flatnonzero(alarm & (ep_t >= start))
    return float(ep_t[hit[0]] - start) if len(hit) else float("nan")


def alarm_events(alarm):
    """Number of separate alarm episodes (rising edges)."""
    alarm = np.asarray(alarm, dtype=bool)
    return int(alarm[0]) + int(np.sum(alarm[1:] & ~alarm[:-1]))


class IntegrityMonitor:
    """Online monitor used for protected navigation.

    While GNSS is trusted, any test alarm stops fusing it: the filter coasts on the IMU and the
    drone is expected to hold or land. GNSS is trusted again only after 3 s of agreement with
    the coasted solution, and only if the outage was short. A course alarm is latched: it is
    never cleared by agreement alone.
    """

    def __init__(self, alpha=ALPHA, window=WINDOW, bias_limit=BIAS_LIMIT, retrust_epochs=30, max_coast=15.0,
                 dual=False):
        self.snap_thr = chi2.ppf(1 - alpha, 2)
        self.win_thr = chi2.ppf(1 - alpha, 2 * window)
        self.agree_thr = chi2.ppf(0.99, 2)
        self.window = deque(maxlen=window)
        self.bias_limit, self.retrust_epochs, self.max_coast = bias_limit, retrust_epochs, max_coast
        self.dual, self.bad_baseline, self.bad_heading, self.bad_course = dual, 0, 0, False
        self.trusted, self.alarm_time, self.since, self.agree = True, float("nan"), 0.0, 0
        self.alarm_source = None   # test that raised the latest alarm: "baseline", "heading", "course" or "imu"
        self.first_source = None   # ... and the one that raised the first alarm of the flight

    def _dual_ok(self, bl_stat, h_nis, c_stat):
        """Track the dual-antenna tests; False once any of them is in alarm."""
        if not self.dual or np.isnan(bl_stat):
            return True
        self.bad_baseline = self.bad_baseline + 1 if bl_stat > DUAL_THRESHOLD else 0
        self.bad_heading = self.bad_heading + 1 if h_nis > DUAL_THRESHOLD else 0
        self.bad_course = bool(np.isfinite(c_stat) and c_stat > DUAL_THRESHOLD)
        return self.bad_baseline < PERSIST and self.bad_heading < PERSIST and not self.bad_course

    def step(self, t, nis, kf, sigma=0.02, bl_stat=float("nan"), h_nis=float("nan"), c_stat=float("nan")):
        """Return True if this fix may be fused."""
        dual_ok = self._dual_ok(bl_stat, h_nis, c_stat)
        if self.trusted:
            if sigma <= PRECISE:          # window test only over fixes claiming RTK-fixed precision
                self.window.append(nis)
            full = len(self.window) == self.window.maxlen
            if (nis > self.snap_thr or (full and sum(self.window) > self.win_thr)
                    or np.abs(kf.x[4:6]).max() > self.bias_limit or not dual_ok):
                self.trusted, self.since, self.agree = False, t, 0
                self.alarm_source = ("baseline" if self.bad_baseline >= PERSIST else
                                     "heading" if self.bad_heading >= PERSIST else
                                     "course" if self.bad_course else "imu")
                if np.isnan(self.alarm_time):
                    self.alarm_time, self.first_source = t, self.alarm_source
                return False
            return True
        if self.alarm_source == "course":
            # A spoofer that reproduces the antenna baseline was dragging the position. Agreement
            # between its fixes and a filter it has already bent proves nothing, so GNSS stays
            # untrusted until something else (an operator, a landing, another sensor) clears it.
            return False
        healthy = dual_ok and (not self.dual or (self.bad_baseline == 0 and self.bad_heading == 0))
        self.agree = self.agree + 1 if nis < self.agree_thr and healthy else 0
        if self.agree >= self.retrust_epochs and t - self.since <= self.max_coast:
            self.trusted = True
            self.window.clear()
            return True
        return False
