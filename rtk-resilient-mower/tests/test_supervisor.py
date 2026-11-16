from mower.gnss import FIXED, FLOAT, NONE
from mower.supervisor import Mode, Supervisor, SupervisorConfig


def test_float_in_open_grass_keeps_mowing():
    s = Supervisor()
    assert s.update(0.0, FIXED, 0.02, clearance=3.0) == Mode.NOMINAL
    assert s.update(1.0, FLOAT, 0.12, clearance=3.0) == Mode.DEGRADED   # far from hazards
    assert 0.2 <= s.speed_limit < 0.5


def test_same_uncertainty_next_to_a_trunk_stops():
    s = Supervisor()
    assert s.update(0.0, FLOAT, 0.12, clearance=0.15) == Mode.HOLD
    assert s.speed_limit == 0.0


def test_hold_resumes_when_fix_returns():
    s = Supervisor()
    s.update(0.0, NONE, 0.08, clearance=0.15)
    assert s.mode == Mode.HOLD
    assert s.update(3.0, FIXED, 0.02, clearance=0.15) == Mode.NOMINAL


def test_hold_timeout_skips_near_hazard_but_retreats_when_lost():
    s = Supervisor(SupervisorConfig(hold_timeout=5.0))
    s.update(0.0, NONE, 0.08, clearance=0.15)
    s.update(5.1, NONE, 0.08, clearance=0.15)
    assert s.hold_expired and s.mode == Mode.DEGRADED     # skip the strip, keep mowing
    lost = Supervisor(SupervisorConfig(hold_timeout=5.0))
    lost.update(0.0, NONE, 0.30, clearance=2.0)
    lost.update(5.1, NONE, 0.30, clearance=2.0)
    assert lost.hold_expired and lost.mode == Mode.RETREAT
    assert lost.update(6.0, FIXED, 0.03) == Mode.NOMINAL
