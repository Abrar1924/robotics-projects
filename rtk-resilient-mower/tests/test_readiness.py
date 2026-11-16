from types import SimpleNamespace

import numpy as np
import shapely
from shapely.geometry import box

from mower.gnss import FIXED, NONE
from mower.readiness import AT_RISK, DEGRADED, READY, readiness
from mower.world import CANOPY_DEEP, suburban_yard

YARD = suburban_yard()


def _survey(deferred=None, dead_zone=None, step=0.05, row=0.25):
    """A synthetic raster drive over the lawn: RTK fixed everywhere except inside `dead_zone`."""
    mow = YARD.mowable_area()
    minx, miny, maxx, maxy = mow.bounds
    xs, ys = np.arange(minx, maxx, step), np.arange(miny, maxy, row)
    X, Y = (a.ravel() for a in np.meshgrid(xs, ys))
    inside = shapely.contains_xy(mow, X, Y)
    X, Y = X[inside], Y[inside]
    q = np.full(len(X), FIXED)
    if dead_zone is not None:
        q[shapely.contains_xy(dead_zone, X, Y)] = NONE
    return SimpleNamespace(true=np.c_[X, Y, np.zeros_like(X)], quality=q, t=np.arange(len(X)) * 0.05,
                           deferred=deferred, mission_time=600.0, coverage=0.95,
                           deferred_area=0.0 if deferred is None else deferred.area)


def test_grid_cells_tile_the_mowable_lawn():
    rep = readiness(YARD, [_survey()])
    assert abs(rep.total - YARD.mowable_area().area) < 0.01 * rep.total


def test_open_sky_everywhere_is_ready_and_go():
    rep = readiness(YARD, [_survey(), _survey()])
    assert rep.share(READY) > 0.97
    assert rep.verdict == "GO"
    assert rep.zones == []


def test_no_fix_slows_the_robot_but_only_skipping_puts_a_zone_at_risk():
    core = next(z.geometry for z in YARD.zones if z.kind == CANOPY_DEEP)
    rep = readiness(YARD, [_survey(dead_zone=core)] * 3)             # driven through on dead reckoning
    assert rep.share(AT_RISK) == 0.0 and rep.share(DEGRADED) > 0.03
    rep = readiness(YARD, [_survey(dead_zone=core, deferred=core)] * 3)  # skipped every time
    assert rep.share(AT_RISK) > 0.03
    assert rep.verdict == "GO WITH MITIGATION"
    biggest = rep.zones[0]
    assert biggest.near == "Under the maple dense core"
    assert "dense canopy" in biggest.cause
    assert abs(biggest.area - core.intersection(YARD.mowable_area()).area) < 6.0   # 1 m cells


def test_strips_deferred_in_most_missions_are_at_risk_even_with_rtk_fixed():
    strip = box(2.0, 1.0, 6.0, 2.0)
    rep = readiness(YARD, [_survey(deferred=strip), _survey(deferred=strip), _survey()])
    risky = rep.cells[rep.klass == AT_RISK]
    assert risky.size > 0 and all(shapely.intersects(c, strip) for c in risky)
