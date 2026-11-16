import numpy as np

from gim.faults import Scenario
from gim.flight import Property, patrol
from gim.sim import fly


def test_patrol_is_constant_speed_and_inside_the_geofence():
    prop = Property()
    t, p, v, a = patrol(prop, speed=5.0, corner_radius=8.0, duration=60.0)
    assert np.allclose(np.hypot(*v.T), 5.0)
    assert np.abs(np.hypot(*a.T) - np.where(np.hypot(*a.T) > 0, 25.0 / 8.0, 0.0)).max() < 1e-9
    assert np.abs(p[:, 0]).max() <= prop.width / 2 - prop.inset + 1e-9
    assert np.abs(p[:, 1]).max() <= prop.height / 2 - prop.inset + 1e-9
    assert np.hypot(*np.diff(p, axis=0).T).max() < 5.0 * 0.02 + 1e-6   # continuous path


def test_filter_is_statistically_consistent_on_clean_flights():
    nis = np.concatenate([fly(Scenario("Nominal"), seed=s).nis for s in (11, 12, 13)])
    assert 1.7 < np.nanmean(nis) < 2.4      # chi-square with 2 dof has mean 2
    errs = [fly(Scenario("Nominal"), seed=14).error.max()]
    assert max(errs) < 0.1
