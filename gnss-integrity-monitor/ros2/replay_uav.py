"""Count fix quality in a real UAV GPS file.

The file is Andrew Tridgell's public UAV log from March 2011:
https://www.samba.org/tridge/UAV/logs/log.nmea
It has ordinary GPS and DGPS. It has no RTK fix and no second antenna,
so the heading check is not applied to it.
"""
from __future__ import annotations

import collections
import sys
from pathlib import Path

PATH = Path(__file__).resolve().parents[1] / "data" / "log.nmea"
NAMES = {"0": "no fix", "1": "GPS", "2": "DGPS"}


def main():
    if not PATH.exists():
        print("FAILED")
        print(f"missing {PATH}")
        sys.exit(1)
    counts = collections.Counter()
    for line in PATH.open(errors="replace"):
        if "GGA" not in line[:7]:
            continue
        parts = line.strip().split(",")
        if len(parts) > 6 and parts[0].endswith("GGA"):
            counts[parts[6]] += 1
    total = sum(counts.values())
    print(f"UAV log: {total} readings")
    for key in sorted(counts):
        print(f"  {NAMES.get(key, key)}: {counts[key]}")
    if total < 1000 or counts.get("4", 0) != 0:
        print("FAILED")
        print("This file should be the 2011 UAV log: many readings, none of them RTK fixed.")
        sys.exit(1)
    print("Heading check not applied: the file has no second antenna.")
    print("PASSED")


if __name__ == "__main__":
    main()
