import math

import numpy as np

from mower.ekf import MowerEKF


def _drive(ekf, rng, x, seconds, gnss=True, v=0.5, slip=0.04, dt=0.05):
    """Straight drive east with constant slip; RTK fixed at 5 Hz when `gnss`."""
    for k in range(int(seconds / dt)):
        x += v * (1 - slip) * dt
        ekf.predict(v, 0.0, dt)
        if gnss and k % 4 == 0:
            ekf.update_position(np.array([x, 0.0]) + rng.normal(0, 0.015, 2), 0.02)
    return x


def test_rtk_fixed_tracks_and_learns_slip():
    rng = np.random.default_rng(1)
    ekf = MowerEKF((0.0, 0.0, 0.0))
    x = _drive(ekf, rng, 0.0, 40.0)
    assert math.hypot(ekf.x[0] - x, ekf.x[1]) < 0.03
    assert abs(ekf.x[4] - 0.96) < 0.01          # odometry scale converges to 1 - slip


def test_outage_grows_uncertainty_but_learned_scale_limits_drift():
    rng = np.random.default_rng(2)
    ekf = MowerEKF((0.0, 0.0, 0.0))
    x = _drive(ekf, rng, 0.0, 40.0)
    sigma_before = ekf.sigma_pos
    x = _drive(ekf, rng, x, 10.0, gnss=False)    # 5 m with no GNSS
    assert ekf.sigma_pos > 3 * sigma_before
    assert abs(ekf.x[0] - x) < 0.05             # slip already learned, so drift stays small


def test_multipath_outlier_is_gated():
    ekf = MowerEKF((0.0, 0.0, 0.0))
    accepted = ekf.update_position(np.array([2.5, -1.0]), 0.12, float_solution=True)
    assert not accepted
    assert math.hypot(ekf.x[0], ekf.x[1]) < 1e-9


def test_consistent_disagreement_reopens_the_filter():
    ekf = MowerEKF((0.0, 0.0, 0.0))
    rng = np.random.default_rng(3)
    for _ in range(12):  # the same 1.2 m offset fix after fix: we drifted, not an outlier
        ekf.update_position(np.array([1.2, 0.0]) + rng.normal(0, 0.015, 2), 0.02)
    assert abs(ekf.x[0] - 1.2) < 0.1
