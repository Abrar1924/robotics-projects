import numpy as np

from mower.gnss import FIXED, FLOAT, NONE, RTKReceiver
from mower.world import suburban_yard


def test_quality_follows_sky_view_and_reconverges():
    yard = suburban_yard()
    rx = RTKReceiver(yard, np.random.default_rng(0))
    assert rx.measure(0.0, 1.0, 1.0)[0] == FIXED       # open sky at the dock
    assert rx.measure(0.2, 21.5, 15.0)[0] == NONE      # under the maple's dense core
    assert rx.measure(0.4, 1.0, 1.0)[0] == FLOAT       # back in the open: ambiguities re-resolving
    later = [rx.measure(20.0 + 0.2 * k, 1.0, 1.0)[0] for k in range(5)]
    assert all(q == FIXED for q in later)


def test_multipath_pushes_solution_away_from_wall():
    yard = suburban_yard()
    # isolate the deterministic push: no random bias, no outlier jumps
    rx = RTKReceiver(yard, np.random.default_rng(4), outlier_prob=0.0, multipath_bias=0.0)
    xs = [rx.measure(0.2 * k, 12.0, 12.0)[1][0] for k in range(200)]  # beside the house's east wall
    assert 12.2 < np.mean(xs) < 12.3                                    # pushed ~0.25 m east, away from it
