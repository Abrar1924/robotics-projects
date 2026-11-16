from collections import Counter
from pathlib import Path

from mower.replay_nmea import fix_losses, read_log, read_qualities, run, summary

LOG = Path("data/MTV.Local1.ublox-F9K.20200206-181434.nmea")


def test_public_drive_has_fixed_and_float_and_the_wall_stops_more():
    qualities = read_qualities(LOG)
    counts = Counter(qualities)
    assert counts[4] == 967
    assert counts[5] == 632
    open_counts, open_stops = run(qualities, 5.0)
    wall_counts, wall_stops = run(qualities, 0.30)
    assert open_counts["NOMINAL"] == 967
    assert wall_counts["NOMINAL"] == 967
    assert wall_stops > open_stops


def test_every_reading_is_counted_in_exactly_one_mode():
    readings = read_log(LOG)
    for clearance in (5.0, 0.30):
        counts, _ = run(readings, clearance)
        assert sum(counts.values()) == len(readings) == 1653


def test_the_receiver_loses_its_fix_for_longer_than_the_hold_timeout():
    readings = read_log(LOG)
    losses = [seconds for _, seconds in fix_losses(readings)]
    s = summary(readings)
    assert len(losses) == s["fix_losses"] == 69
    assert max(losses) == s["longest_loss_s"] == 73.0
    assert s["losses_over_timeout"] == 19
    # time stamps come from the receiver: the log has silences, so it is longer than its reading count
    assert readings[-1].t > len(readings)


if __name__ == "__main__":
    test_public_drive_has_fixed_and_float_and_the_wall_stops_more()
    print("PASSED")
