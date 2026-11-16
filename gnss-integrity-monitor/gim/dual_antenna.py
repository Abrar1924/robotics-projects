"""Dual-antenna GNSS: the moving-base baseline, heading from it, and the integrity tests they allow.

A receiver with two RF inputs can solve for the vector between two antennas on the airframe
(moving-base RTK). The airframe fixes that vector's length, and its direction is where the
nose points. That gives three checks, ordered from the easiest attacker to the hardest:

  baseline  The measured antenna separation must equal the known length. A spoofer that
            broadcasts from a single antenna feeds both antennas the same signals from the
            same direction, so both solve to the same point and the measured baseline
            collapses towards zero, however gently the spoofer drags the position.
  heading   The baseline's direction is a heading measurement that needs no magnetometer.
            It must agree with the gyro-propagated heading. A spoofer that fakes a baseline
            of the right length but cannot follow the drone's yaw is caught at the next turn.
  course    Heading is where the nose points; course is where the vehicle moves. The flight
            controller sets the angle between them (zero when flying nose-first, or a
            commanded yaw offset). A spoofer that reproduces the baseline perfectly but drags
            the position sideways bends the filter's course away from the heading.

The course test is slow against a gentle drag and blind to a drag along the direction of
travel; that last gap needs another reference, such as visual-inertial odometry.

The simulation is 2-D and level. A real implementation tests the 3-D baseline length,
which does not change with roll or pitch.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np
from scipy.stats import chi2

BASELINE = 0.20   # m: antenna separation; 0.2 m is claimed to be enough for a sub-250 g drone
SIGMA_B = 0.005   # m: 1-sigma error per axis of a fixed moving-base solution (assumption);
                  # 5 mm across 0.20 m is 1.4 degrees of heading noise
ALPHA = 1e-5      # per-epoch false-alarm probability of each test before persistence
PERSIST = 3       # consecutive epochs over threshold before an alarm (0.2 s at 10 Hz)
THRESHOLD = float(chi2.ppf(1 - ALPHA, 1))   # every statistic here is chi-square with 1 dof

SIGMA_SLIP = math.radians(1.0)   # rad: 1-sigma budget for the drift angle the controller does not
                                 # remove (gusts, tracking error) in nose-first flight (assumption)
SLIP_TAU = 2.0                   # s: correlation time of the simulated drift-angle error
COURSE_WINDOW = 30               # epochs averaged by the course test (3 s at 10 Hz)
COURSE_MIN = 20                  # valid epochs the window needs before it may alarm
MIN_SPEED = 1.0                  # m/s: course is undefined when the vehicle is nearly stationary


def wrap(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Gyro:
    """Yaw-rate gyro at 50 Hz: consumer MEMS after an at-rest calibration.

    Residual bias ~0.1 deg/s with a slow random walk, white noise ~0.06 deg/s per sample.
    """

    def __init__(self, rng, noise=0.001, bias_sigma=0.002, bias_walk=1e-4):
        self.rng, self.noise, self.bias_walk = rng, noise, bias_walk
        self.bias = float(rng.normal(0.0, bias_sigma))

    def sample(self, rate_true, dt):
        self.bias += self.bias_walk * math.sqrt(dt) * float(self.rng.normal())
        return rate_true + self.bias + self.noise * float(self.rng.normal())


class MovingBaseRTK:
    """Antenna-to-antenna vector (primary to secondary antenna) in the local level frame."""

    def __init__(self, rng, length=BASELINE, sigma=SIGMA_B):
        self.rng, self.length, self.sigma = rng, length, sigma

    def measure(self, heading):
        b = self.length * np.array([math.cos(heading), math.sin(heading)])
        return b + self.rng.normal(0.0, self.sigma, 2), self.sigma


def baseline_stat(b, sigma, length=BASELINE):
    """Squared, normalised error of the measured baseline length: chi-square(1) when healthy."""
    return ((float(np.hypot(*b)) - length) / sigma) ** 2


class HeadingFilter:
    """Two-state Kalman filter [heading, gyro bias], driven by the gyro, corrected by GNSS heading.

    Heading fixes that fail either test are not fused, so a corrupted baseline cannot drag the
    heading along with it. After `reacquire` rejected fixes in a row whose baseline length is
    healthy, the filter re-initialises from GNSS (a long disturbance, not a single outlier).
    """

    def __init__(self, heading0, heading_sigma=0.05, bias_sigma=0.003, rate_noise=0.001,
                 bias_walk=1e-4, reacquire=30):
        self.x = np.array([heading0, 0.0])
        self.P = np.diag([heading_sigma**2, bias_sigma**2])
        self.rate_noise, self.bias_walk, self.reacquire = rate_noise, bias_walk, reacquire
        self.rejected = 0

    def predict(self, rate_meas, dt):
        self.x[0] = wrap(self.x[0] + (rate_meas - self.x[1]) * dt)
        F = np.array([[1.0, -dt], [0.0, 1.0]])
        Q = np.diag([(2.0 * self.rate_noise * dt) ** 2, self.bias_walk**2 * dt])
        self.P = F @ self.P @ F.T + Q

    def innovation(self, heading_meas, sigma):
        nu = wrap(heading_meas - self.x[0])
        S = self.P[0, 0] + sigma * sigma
        return nu, S, nu * nu / S

    def step(self, b, sigma_b, length=BASELINE, reacquire=True):
        """Test one baseline fix and fuse its heading if it passes.

        `reacquire=False` keeps the filter on the gyro however long fixes are rejected: the caller
        sets it while an integrity alarm is active, so a spoofed heading is never adopted.
        Returns (baseline statistic, heading NIS); both chi-square(1) when healthy.
        """
        len_stat = baseline_stat(b, sigma_b, length)
        sigma_h = sigma_b / length
        heading = math.atan2(b[1], b[0])
        nu, S, nis = self.innovation(heading, sigma_h)
        if len_stat < THRESHOLD and nis < THRESHOLD:
            K = self.P[:, 0] / S
            self.x = self.x + K * nu
            self.x[0] = wrap(self.x[0])
            I_KH = np.eye(2) - np.outer(K, [1.0, 0.0])
            self.P = I_KH @ self.P @ I_KH.T + sigma_h * sigma_h * np.outer(K, K)
            self.rejected = 0
        elif len_stat < THRESHOLD and reacquire:
            self.rejected += 1
            if self.rejected >= self.reacquire:
                self.x[0] = heading
                self.P[0, 0] = max(self.P[0, 0], 4.0 * sigma_h * sigma_h)
                self.rejected = 0
        return len_stat, nis


class CourseCheck:
    """Windowed test that the direction of travel agrees with where the nose points.

    residual = filter course - antenna heading + expected drift angle
    `expected_drift` is the drift angle the flight controller commands (heading minus course):
    zero for nose-first flight, the yaw offset when the nose is deliberately held off the track.
    The statistic is the squared mean residual over the last 3 s, normalised by
      antenna heading noise (white, so it averages down with the number of samples),
      the drift-angle budget SIGMA_SLIP (correlated, so it is not averaged down), and
      the filter's own course uncertainty.
    It is chi-square(1) or smaller on healthy flights.
    """

    def __init__(self, window=COURSE_WINDOW, min_valid=COURSE_MIN, sigma_slip=SIGMA_SLIP):
        self.residuals = deque(maxlen=window)
        self.min_valid, self.sigma_slip = min_valid, sigma_slip
        self.mean = 0.0   # latest windowed mean residual (rad)

    def step(self, velocity, vel_cov, heading, sigma_h, expected_drift=0.0):
        """Add one epoch. `heading` is NaN when the antenna heading is unusable.

        Returns (residual, statistic); either is NaN when it cannot be formed.
        """
        speed = float(np.hypot(velocity[0], velocity[1]))
        if speed < MIN_SPEED or not np.isfinite(heading):
            self.residuals.append(np.nan)
            return float("nan"), float("nan")
        course = math.atan2(velocity[1], velocity[0])
        residual = wrap(course - heading + expected_drift)
        self.residuals.append(residual)
        valid = [r for r in self.residuals if np.isfinite(r)]
        if len(valid) < self.min_valid:
            return residual, float("nan")
        # course variance: the filter's cross-track velocity variance over speed squared
        px, py = -velocity[1] / speed, velocity[0] / speed
        var_course = (px * px * vel_cov[0][0] + py * py * vel_cov[1][1] + 2 * px * py * vel_cov[0][1]) / speed**2
        var = sigma_h * sigma_h / len(valid) + self.sigma_slip**2 + var_course
        self.mean = sum(valid) / len(valid)
        return residual, self.mean * self.mean / var


def gauss_markov(rng, n, sigma, tau, dt):
    """First-order Gauss-Markov sequence of length n (stationary 1-sigma `sigma`, time constant `tau`)."""
    phi = math.exp(-dt / tau)
    drive = math.sqrt(1.0 - phi * phi) * sigma * rng.normal(size=n)
    out = np.empty(n)
    out[0] = sigma * rng.normal()
    for k in range(1, n):
        out[k] = phi * out[k - 1] + drive[k]
    return out


def persistent(flags, n=PERSIST):
    """True where `flags` has been True for at least n consecutive epochs."""
    run, out = 0, np.zeros(len(flags), dtype=bool)
    for i, f in enumerate(flags):
        run = run + 1 if f else 0
        out[i] = run >= n
    return out
