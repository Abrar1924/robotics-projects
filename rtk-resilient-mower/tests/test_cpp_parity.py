"""The C++ supervisor (the one the ROS 2 node runs) must answer exactly like the Python one.

ros2/mower_supervisor_cpp is compiled here with g++ (no ROS needed) and fed the same readings:
the real receiver log at two clearances, and 20,000 random readings that reach every branch.
"""
from __future__ import annotations

import random
import shutil
import subprocess
from pathlib import Path

import pytest

from mower.replay_nmea import DEFAULT_SIGMA, SIGMA, quality_code, read_log
from mower.supervisor import Mode, Supervisor

SRC = Path(__file__).resolve().parents[1] / "ros2" / "mower_supervisor_cpp" / "src"
LOG = Path(__file__).resolve().parents[1] / "data" / "MTV.Local1.ublox-F9K.20200206-181434.nmea"

pytestmark = pytest.mark.skipif(shutil.which("g++") is None, reason="g++ is not installed")


@pytest.fixture(scope="module")
def replay(tmp_path_factory):
    binary = tmp_path_factory.mktemp("cpp") / "replay"
    subprocess.run(["g++", "-std=c++17", "-O2", str(SRC / "supervisor.cpp"), str(SRC / "replay_main.cpp"),
                    "-o", str(binary)], check=True)
    return binary


def compare(binary, rows):
    """rows: (t, GGA quality, sigma, clearance). Both supervisors must agree on every one."""
    text = "".join(f"{t!r} {gga} {sigma!r} {clearance!r}\n" for t, gga, sigma, clearance in rows)
    out = subprocess.run([str(binary)], input=text, capture_output=True, text=True, check=True).stdout.splitlines()
    assert len(out) == len(rows)
    sup, seen = Supervisor(), set()
    for (t, gga, sigma, clearance), line in zip(rows, out):
        mode = sup.update(t, quality_code(gga), sigma, clearance)
        name, speed, expired = line.split()
        assert name == Mode(mode).name
        assert float(speed) == pytest.approx(sup.speed_limit, abs=1e-12)
        assert bool(int(expired)) == sup.hold_expired
        seen.add((Mode(mode), sup.hold_expired))
    return seen


@pytest.mark.parametrize("clearance", [5.0, 0.30])
def test_real_receiver_log(replay, clearance):
    rows = [(r.t, r.quality, SIGMA.get(r.quality, DEFAULT_SIGMA), clearance) for r in read_log(LOG)]
    assert len(rows) == 1653
    compare(replay, rows)


def test_random_readings_reach_every_mode_and_a_timed_out_hold(replay):
    rng = random.Random(7)
    rows, t = [], 0.0
    for _ in range(20000):
        t += rng.choice((0.1, 0.2, 1.0, 3.0))
        gga = rng.choice((4, 4, 4, 5, 5, 2, 1, 0))
        sigma = rng.choice((0.01, 0.02, 0.039, 0.04, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.6, 1.0))
        clearance = rng.choice((0.05, 0.15, 0.3, 0.6, 1.0, 3.0, 10.0, float("inf")))
        rows.append((t, gga, sigma, clearance))
    seen = compare(replay, rows)
    assert {m for m, _ in seen} == set(Mode)
    assert any(expired for _, expired in seen)
