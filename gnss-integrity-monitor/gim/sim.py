"""Fly one patrol with a GNSS fault, run the filter, optionally protected by the monitor."""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .detectors import IntegrityMonitor
from .dual_antenna import (BASELINE, SIGMA_SLIP, SLIP_TAU, THRESHOLD, CourseCheck, Gyro, HeadingFilter,
                           MovingBaseRTK, gauss_markov, wrap)
from .faults import Scenario
from .filter import GnssInsKF
from .flight import Property, patrol
from .sensors import Accelerometer, RTKReceiver

IMU_DT, GNSS_EVERY = 0.02, 5   # 50 Hz accelerometer, 10 Hz GNSS
REWIND = 3.0                   # s: on alarm, restore the filter from before the suspected onset
REWIND_BASELINE = 0.5          # s: a baseline-length alarm fires within 0.2 s of onset, so look back less
COURSE_BIAS_SIGMA = 0.03       # m/s^2: accelerometer-bias uncertainty restored after a course alarm
DECIMATE_DEGRADED = True       # fuse degraded (float) fixes at 1 Hz instead of 10 Hz


@dataclass
class FlightLog:
    scenario: Scenario
    seed: int
    t: np.ndarray        # IMU-rate time (s)
    truth: np.ndarray    # true position along the reference (N, 2)
    est: np.ndarray      # filter position (N, 2)
    ep_t: np.ndarray     # GNSS epoch times (M,)
    ep_ok: np.ndarray    # receiver produced a fix
    nis: np.ndarray      # normalised innovation squared (NaN without a fix)
    innov: np.ndarray    # innovations (M, 2)
    s_trace: np.ndarray  # trace of the innovation covariance
    bias: np.ndarray     # filter's accelerometer-bias estimate at each epoch (M, 2)
    sigma: np.ndarray    # accuracy the receiver reported (NaN without a fix)
    used: np.ndarray     # fix fused into the filter
    bl_len: np.ndarray   # measured antenna-to-antenna baseline length (m; NaN without a fix)
    bl_stat: np.ndarray  # baseline-length test statistic, chi-square(1) when healthy
    h_nis: np.ndarray    # GNSS-heading vs gyro-heading NIS, chi-square(1) when healthy
    c_res: np.ndarray    # course minus antenna heading, less the commanded drift angle (rad)
    c_stat: np.ndarray   # 3 s course-vs-heading statistic, chi-square(1) or smaller when healthy
    alarm_time: float    # first integrity alarm (protected runs), NaN otherwise
    alarm_source: str = ""   # test that raised it: "imu", "baseline", "heading" or "course"

    @property
    def error(self):
        """Navigation error. With a perfect tracking controller this is also how far the
        drone is physically dragged off its patrol path: it flies to keep the estimate on
        the path, so its true position ends up at reference - error."""
        return np.hypot(*(self.est - self.truth).T)

    @property
    def flown(self):
        return self.truth - (self.est - self.truth)


def fly(scenario=Scenario("Nominal"), seed=0, protect=False, duration=180.0, prop=Property(), dual=False,
        yaw_offset=0.0, drift_aware=True, tap=None):
    """Fly one patrol. `protect` puts the integrity monitor in the loop; `dual` lets it also use
    the dual-antenna baseline, heading and course tests (they are recorded either way).

    `yaw_offset` holds the nose that many radians off the direction of travel for the whole
    flight (a camera pointed off-track, or a deliberate crab). With `drift_aware` the course test
    is told that commanded angle; without it the test assumes nose-first flight.

    `tap` is an optional list. It receives every input the dual-antenna tests consume, in order
    (see gim/export.py), so the same flight can be replayed through the C++ monitor.
    """
    rng = np.random.default_rng(seed)
    t, p, v, a = patrol(prop, duration=duration, dt=IMU_DT)
    imu, rx = Accelerometer(rng), RTKReceiver(rng)
    kf = GnssInsKF(p[0] + rng.normal(0.0, 0.02, 2), v[0] + rng.normal(0.0, 0.05, 2))
    # Dual-antenna sensors draw from their own stream, so single-antenna results stay reproducible.
    rng2 = np.random.default_rng([seed, 2])
    # Nose-first flight: the controller holds the nose on the direction of travel (plus any
    # commanded offset). Gusts and tracking error leave a small, slowly varying drift angle.
    slip = gauss_markov(rng2, len(t), SIGMA_SLIP, SLIP_TAU, IMU_DT)
    heading = np.arctan2(v[:, 1], v[:, 0]) + yaw_offset + slip
    gyro, mbase = Gyro(rng2), MovingBaseRTK(rng2)
    hf = HeadingFilter(heading[0] + rng2.normal(0.0, 0.02))
    course_check = CourseCheck()
    expected_drift = yaw_offset if drift_aware else 0.0
    if tap is not None:
        tap.append(("H", float(hf.x[0])))
    monitor = IntegrityMonitor(dual=dual) if protect else None
    memory, bmemory = {}, {}
    est = np.zeros_like(p)
    rec = {k: [] for k in ("ep_t", "ep_ok", "nis", "innov", "s_trace", "bias", "sigma", "used",
                           "bl_len", "bl_stat", "h_nis", "c_res", "c_stat")}
    imu_log = deque(maxlen=int(REWIND / IMU_DT) + 20)                  # (step, accel sample)
    snapshots = deque(maxlen=int(REWIND / (IMU_DT * GNSS_EVERY)) + 5)  # (step, x, P)
    for k in range(len(t)):
        if k > 0:
            # an accelerometer integrates over the interval: feed the interval-average acceleration
            accel = imu.sample((v[k] - v[k - 1]) / IMU_DT, IMU_DT)
            kf.predict(accel, IMU_DT)
            imu_log.append((k, accel))
            rate = gyro.sample(wrap(heading[k] - heading[k - 1]) / IMU_DT, IMU_DT)
            hf.predict(rate, IMU_DT)
            if tap is not None:
                tap.append(("G", float(t[k]), float(rate), IMU_DT))
        if k % GNSS_EVERY == 0:
            z, sigma = rx.measure(p[k])
            z, sigma = scenario.apply(t[k], z, sigma, rng, memory)
            b, sigma_b = mbase.measure(heading[k])
            b, sigma_b = scenario.apply_baseline(t[k], b, sigma_b, rng2, bmemory)
            if tap is not None:
                tap.append(("E", float(t[k]), None if b is None else (float(b[0]), float(b[1])), sigma_b,
                            kf.x[2:4].copy(), kf.P[2:4, 2:4].copy(), expected_drift))
            if b is None:
                bl_len = bl_stat = h_nis = np.nan
                c_res, c_stat = course_check.step(kf.x[2:4], kf.P[2:4, 2:4], np.nan, np.nan)
            else:
                bl_len = float(np.hypot(*b))
                bl_stat, h_nis = hf.step(b, sigma_b, reacquire=monitor is None or monitor.trusted)
                # a heading taken from a baseline of the wrong length means nothing
                gnss_heading = math.atan2(b[1], b[0]) if bl_stat < THRESHOLD else np.nan
                c_res, c_stat = course_check.step(kf.x[2:4], kf.P[2:4, 2:4], gnss_heading,
                                                  sigma_b / BASELINE, expected_drift)
            if z is None:
                nu, s_tr, nis, used, sigma = np.full(2, np.nan), np.nan, np.nan, False, np.nan
            else:
                nu, S, nis = kf.innovation(z, sigma)
                s_tr = float(np.trace(S))
                was_trusted = monitor.trusted if monitor else True
                used = monitor.step(t[k], nis, kf, sigma, bl_stat, h_nis, c_stat) if monitor else True
                # Degraded (float) fixes have correlated errors: fusing them at 10 Hz as if they
                # were independent makes the filter overconfident. Fuse them at 1 Hz instead.
                if used and DECIMATE_DEGRADED and sigma > 0.05 and (k // GNSS_EVERY) % 10:
                    used = False
                if used:
                    kf.update(z, sigma)
                    if sigma > 0.05:
                        kf.floor_position(0.5 * sigma)
                elif monitor and was_trusted and not monitor.trusted and snapshots:
                    # Alarm just raised: fixes before it may already have been corrupted.
                    # Restore the filter from REWIND seconds ago and replay the IMU only.
                    back = REWIND_BASELINE if monitor.alarm_source == "baseline" else REWIND
                    target = k - int(back / IMU_DT)
                    k0, x0, P0 = next((s for s in reversed(snapshots) if s[0] <= target), snapshots[0])
                    kf.x, kf.P = x0.copy(), P0.copy()
                    for kk, accel in imu_log:
                        if kk > k0:
                            kf.predict(accel, IMU_DT)
                    if monitor.alarm_source == "course":
                        # A course alarm means the cross-track velocity (and the accelerometer bias
                        # that absorbed the drag) can no longer be trusted: say so in the covariance,
                        # so the nose-first constraint below can correct them.
                        speed = float(np.hypot(*kf.x[2:4]))
                        kf.P[2:4, 2:4] += (speed * course_check.mean) ** 2 * np.eye(2)
                        kf.P[4:6, 4:6] += COURSE_BIAS_SIGMA**2 * np.eye(2)
            if monitor and dual and not monitor.trusted and (k // GNSS_EVERY) % 10 == 0:
                # Heading-aided coast: the gyro-propagated heading still says which way the drone is
                # travelling, so hold the filter's cross-track velocity near zero while it coasts.
                # The drift-angle error is correlated over seconds, so apply this at 1 Hz with a
                # doubled budget instead of pretending 10 Hz samples are independent.
                speed = float(np.hypot(*kf.x[2:4]))
                if speed >= 1.0:
                    kf.constrain_lateral(hf.x[0] - expected_drift,
                                         speed * math.sqrt((2.0 * SIGMA_SLIP) ** 2 + float(hf.P[0, 0])))
            snapshots.append((k, kf.x.copy(), kf.P.copy()))
            rec["ep_t"].append(t[k])
            rec["ep_ok"].append(z is not None)
            rec["nis"].append(nis)
            rec["innov"].append(nu)
            rec["s_trace"].append(s_tr)
            rec["bias"].append(kf.x[4:6].copy())
            rec["sigma"].append(sigma)
            rec["used"].append(used)
            rec["bl_len"].append(bl_len)
            rec["bl_stat"].append(bl_stat)
            rec["h_nis"].append(h_nis)
            rec["c_res"].append(c_res)
            rec["c_stat"].append(c_stat)
        est[k] = kf.x[0:2]
    arr = {k: np.asarray(v) for k, v in rec.items()}
    return FlightLog(scenario=scenario, seed=seed, t=t, truth=p, est=est,
                     alarm_time=monitor.alarm_time if monitor else float("nan"),
                     alarm_source=(monitor.first_source or "") if monitor else "", **arr)
