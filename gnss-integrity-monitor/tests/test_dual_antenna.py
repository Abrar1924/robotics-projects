import math

import numpy as np

from gim.detectors import classical_alarms, dual_alarms, first_alarm
from gim.faults import Scenario
from gim.sim import fly

GENTLE = dict(kind="drift_spoof", magnitude=0.03)   # below the IMU tests' detection floor


def test_dual_antenna_statistics_are_consistent_on_clean_flights():
    logs = [fly(Scenario("Nominal"), seed=s) for s in (31, 32)]
    bl = np.concatenate([lg.bl_stat for lg in logs])
    hd = np.concatenate([lg.h_nis for lg in logs])
    co = np.concatenate([lg.c_stat for lg in logs])
    assert 0.7 < np.nanmean(bl) < 1.3          # chi-square with 1 dof has mean 1
    assert 0.7 < np.nanmean(hd) < 1.4
    assert np.nanmean(co) < 1.3                # conservative by design: at or below 1
    assert not any(a.any() for lg in logs for a in dual_alarms(lg).values())


def test_single_transmitter_spoof_below_the_imu_floor_collapses_the_baseline():
    sc = Scenario("gentle", **GENTLE)
    log = fly(sc, seed=33)
    imu = classical_alarms(log)
    assert not any((a & (log.ep_t >= sc.start)).any() for a in imu.values())   # the IMU tests miss it
    assert first_alarm(log.ep_t, dual_alarms(log)["baseline"], sc.start) <= 0.3


def test_frozen_baseline_array_is_caught_by_the_heading_test_at_a_turn():
    sc = Scenario("frozen", array="frozen", **GENTLE)
    log = fly(sc, seed=34)
    alarms = dual_alarms(log)
    assert np.isnan(first_alarm(log.ep_t, alarms["baseline"], sc.start))     # length looks right
    assert first_alarm(log.ep_t, alarms["heading"], sc.start) < 10.0          # next turn gives it away


def test_attitude_aware_array_passes_baseline_and_heading_but_bends_the_course():
    """The hardest attacker modelled: the baseline is reproduced perfectly while the position is
    dragged. Only the course test sees it, late: after one to two legs across the drag."""
    sc = Scenario("tracking", array="tracking", **GENTLE)
    delays = []
    for seed in (35, 36, 37):
        log = fly(sc, seed=seed)
        during = (log.ep_t >= sc.start) & (log.ep_t <= sc.start + 60.0)
        assert not any((a & during).any() for a in classical_alarms(log).values())
        alarms = dual_alarms(log)
        assert not alarms["baseline"].any() and not alarms["heading"].any()
        delays.append(first_alarm(log.ep_t, alarms["course"], sc.start))
    assert all(5.0 < d < 30.0 for d in delays)


def test_course_test_needs_the_commanded_drift_angle_when_the_nose_is_held_off_track():
    """Heading is where the nose points, course is where the drone moves. With the nose held
    20 degrees off the track, a test that assumes nose-first flight alarms on a clean flight;
    told the commanded angle, it stays quiet and still catches the spoof."""
    offset = math.radians(20.0)
    naive = fly(Scenario("Nominal"), seed=38, yaw_offset=offset, drift_aware=False)
    assert dual_alarms(naive)["course"].any()
    aware = fly(Scenario("Nominal"), seed=38, yaw_offset=offset)
    assert not any(a.any() for a in dual_alarms(aware).values())
    sc = Scenario("tracking", array="tracking", **GENTLE)
    spoofed = fly(sc, seed=38, yaw_offset=offset)
    assert first_alarm(spoofed.ep_t, dual_alarms(spoofed)["course"], sc.start) < 30.0


def test_honest_float_is_not_flagged_and_multipath_is():
    flt = Scenario("float", "float", duration=30.0)
    log = fly(flt, seed=36)
    assert not any(a.any() for a in dual_alarms(log).values())
    mp = Scenario("multipath", "multipath", duration=10.0, magnitude=1.5, direction=(0.6, 0.8))
    log = fly(mp, seed=36)
    alarms = dual_alarms(log)
    assert first_alarm(log.ep_t, alarms["baseline"] | alarms["heading"], mp.start) < 1.0


def test_dual_antenna_monitor_stops_a_gentle_single_transmitter_spoof():
    sc = Scenario("gentle", **GENTLE)
    free = fly(sc, seed=37)
    guarded = fly(sc, seed=37, protect=True, dual=True)
    assert free.error[free.t >= sc.start + 30].max() > 10.0                  # unprotected: dragged > 10 m
    assert guarded.alarm_source == "baseline"
    assert guarded.alarm_time - sc.start <= 0.3
    window = (guarded.t >= sc.start) & (guarded.t <= guarded.alarm_time + 10)
    assert guarded.error[window].max() < 1.5


def test_dual_antenna_monitor_raises_no_alarm_on_a_clean_flight():
    guarded = fly(Scenario("Nominal"), seed=39, protect=True, dual=True)
    assert math.isnan(guarded.alarm_time)


def test_course_alarm_is_latched_and_the_heading_aided_coast_limits_the_drag():
    """After a course alarm the monitor must not re-trust fixes from a spoofer it cannot tell
    apart by agreement, and the nose-first constraint must stop the cross-track drag growing."""
    sc = Scenario("tracking", array="tracking", **GENTLE)
    free = fly(sc, seed=35)
    guarded = fly(sc, seed=35, protect=True, dual=True)
    assert guarded.alarm_source == "course"
    after = guarded.ep_t > guarded.alarm_time
    assert not guarded.used[after].any()                                      # latched: no fix fused again
    k = np.searchsorted(guarded.t, guarded.alarm_time + 10.0)
    assert guarded.error[k] < 0.6 * free.error[k]                             # well under the unprotected drag
