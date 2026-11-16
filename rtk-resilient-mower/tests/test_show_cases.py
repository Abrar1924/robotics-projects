from mower.show_cases import situations, decide
from mower.supervisor import Mode, Supervisor


def test_three_places_get_the_right_answer():
    for case in situations():
        mode, speed, _expired = decide(case["status"], case["sigma_m"], case["clearance_m"])
        assert mode == case["mode"], case["name"]
        if "speed_mps" in case:
            assert abs(speed - case["speed_mps"]) < 1e-9
        if case["name"] == "under a tree":
            assert 0.20 <= speed < 0.50


def test_waiting_by_the_wall_then_skips_the_strip():
    sup = Supervisor()
    mode, speed, expired = decide(0, 0.20, 0.30, t=0.0, supervisor=sup)
    assert mode == Mode.HOLD and speed == 0.0 and not expired
    mode, speed, expired = decide(0, 0.20, 0.30, t=11.0, supervisor=sup)
    assert expired
    assert mode == Mode.DEGRADED
    assert speed > 0.0
