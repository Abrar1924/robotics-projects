import numpy as np

from gim.detectors import classical_alarms, first_alarm
from gim.faults import Scenario
from gim.sim import fly


def test_clean_flight_raises_no_classical_alarm():
    log = fly(Scenario("Nominal"), seed=21)
    assert not any(a.any() for a in classical_alarms(log).values())


def test_honest_degradation_is_not_an_attack():
    """No alarm while the receiver honestly reports float accuracy, nor when RTK fixed returns.
    (Background false alarms elsewhere in a flight are measured separately, per hour.)"""
    sc = Scenario("float", "float", duration=30.0)
    for seed in range(22, 27):
        log = fly(sc, seed=seed)
        during = (log.ep_t >= sc.start) & (log.ep_t <= sc.start + sc.duration + 10.0)
        assert not any((a & during).any() for a in classical_alarms(log).values())


def test_jump_spoof_is_caught_on_the_first_fix():
    sc = Scenario("jump", "jump_spoof", magnitude=8.0)
    log = fly(sc, seed=23)
    assert first_alarm(log.ep_t, classical_alarms(log)["snapshot"], sc.start) == 0.0


def test_monitor_bounds_a_drift_spoof():
    sc = Scenario("drift", "drift_spoof", magnitude=0.2)
    free, guarded = fly(sc, seed=24), fly(sc, seed=24, protect=True)
    after = free.t >= sc.start + 30
    assert free.error[after].max() > 20.0                        # unprotected: dragged tens of metres
    window = (guarded.t >= sc.start) & (guarded.t <= guarded.alarm_time + 10)
    assert guarded.alarm_time - sc.start < 5.0
    assert guarded.error[window].max() < 3.0                      # protected: alarm, rewind, hold
