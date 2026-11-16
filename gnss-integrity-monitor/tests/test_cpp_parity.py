"""The C++ monitor must make the same decisions as the Python tests, epoch for epoch.

Each flight is simulated in Python with its inputs tapped, the same inputs are replayed through
ros2/gim_monitor (compiled here with g++, no ROS needed), and every statistic is compared.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from gim.detectors import dual_alarms
from gim.export import lines, record, replay_cases

PKG = Path(__file__).resolve().parents[1] / "ros2" / "gim_monitor"

pytestmark = pytest.mark.skipif(shutil.which("g++") is None, reason="g++ is not installed")


@pytest.fixture(scope="module")
def replay(tmp_path_factory):
    binary = tmp_path_factory.mktemp("cpp") / "replay"
    subprocess.run(["g++", "-std=c++17", "-O2", f"-I{PKG / 'include'}", str(PKG / "src" / "replay_main.cpp"),
                    "-o", str(binary)], check=True)
    return binary


def run(binary, tap, *flags):
    out = subprocess.run([str(binary), *flags], input="\n".join(lines(tap)) + "\n", capture_output=True,
                         text=True, check=True).stdout.splitlines()
    rows = [r.split() for r in out]
    return np.array([[float(x) for x in r[:9]] for r in rows]), [r[9] for r in rows]


@pytest.mark.parametrize("slug,scenario", replay_cases())
def test_cpp_statistics_and_alarms_match_python(replay, slug, scenario):
    log, tap = record(scenario)
    stats, _ = run(replay, tap)
    assert len(stats) == len(log.ep_t)
    for column, name in ((1, "bl_stat"), (2, "h_nis"), (3, "c_res"), (4, "c_stat")):
        np.testing.assert_allclose(stats[:, column], getattr(log, name), rtol=1e-6, atol=1e-9, equal_nan=True,
                                   err_msg=f"{slug}: {name}")
    alarms = dual_alarms(log)
    for column, name in ((5, "baseline"), (6, "heading"), (7, "course")):
        assert np.array_equal(stats[:, column].astype(bool), alarms[name]), f"{slug}: {name} alarm"


def test_latched_monitor_keeps_gnss_untrusted_after_a_course_alarm(replay):
    slug, scenario = replay_cases()[3]
    log, tap = record(scenario)
    stats, source = run(replay, tap, "--latch")
    trusted = stats[:, 8].astype(bool)
    first = int(np.flatnonzero(~trusted)[0])
    assert source[first] == "course"
    assert log.ep_t[first] > scenario.start
    assert not trusted[first:].any()          # never trusted again without a reset


def test_latched_monitor_stays_trusted_on_a_clean_flight(replay):
    slug, scenario = replay_cases()[0]
    _, tap = record(scenario)
    stats, _ = run(replay, tap, "--latch")
    assert stats[:, 8].all()
