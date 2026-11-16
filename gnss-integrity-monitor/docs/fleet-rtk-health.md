# Design note: fleet RTK-health pipeline (not built)

**Status:** design only. It describes how the per-robot checks in this repo and in
[`rtk-resilient-mower`](../../rtk-resilient-mower) would scale to a fleet of drones and mowers.

## Problem

One robot can tell that *its* GNSS is wrong. It can't tell *why*. Three causes look alike from a
single robot and need different responses:

| Cause | What one robot sees | What the fleet sees | Response |
|---|---|---|---|
| Local sky view (trees, walls) | Float or no fix in the same spots every mission | Only that yard, repeatably | Readiness report and per-yard mitigation |
| Bad correction source (a reference station with stale or wrong data) | Float, or fixes that fail integrity tests | Every robot using that station, across many yards, at once | Switch station, flag it to the network operator |
| Interference (jamming or spoofing) | Integrity alarms, baseline collapse, fix loss | Robots clustered in space and time, whatever station they use | Hold or land in the area, alert, log evidence |

## Data flow

```
robot (10 Hz state, 1 Hz summary)
  └─ MQTT over TLS ─▶ ingest gateway ─▶ Kafka
                                          ├─ rtk.status.v1     per robot, 1 Hz: fix type, satellites, correction age, station id, position
                                          ├─ rtk.integrity.v1  events: alarm source (snapshot, window, bias, baseline, heading), test values
                                          └─ rtk.mission.v1    end of mission: coverage, deferred area, readiness cells
                    stream processors (Kafka Streams or Flink)
                                          ├─ station health: per station, 5 min tumbling windows
                                          ├─ interference: alarms clustered by geohash and 60 s windows
                                          └─ yard readiness: rolling readiness map per property
                    sinks ─▶ time-series store (dashboards), alert router, object store (raw logs for replay)
```

## Event schema (rtk.integrity.v1)

```json
{"robot_id": "mower-0142", "t": "2026-11-15T18:03:21.400Z", "lat": 37.3541, "lon": -121.9552,
 "station_id": "GEOD-xxxx", "fix": "RTK_FIXED", "source": "baseline",
 "values": {"baseline_len_m": 0.004, "baseline_stat": 1520.3, "heading_nis": 845.1, "nis": 1.8},
 "firmware": "2.3.1"}
```

Keyed by `robot_id` for per-robot ordering; the interference processor re-keys by geohash.

## Detection logic

- **Station health:** for each station, the share of robots whose fix rate dropped or that raised
  integrity alarms in the window, compared with the same robots' history. Many robots, many
  yards, one shared station and simultaneous onset point at the station, not the sky.
- **Interference:** baseline-collapse or snapshot alarms from two or more robots within 500 m and
  60 s, independent of station, open an incident. Single-robot alarms stay per-robot.
- **Yard readiness:** each mission's readiness cells update a per-property map, so the verdict
  improves with data instead of resting on one survey.

## Targets

- Alarm-to-alert latency under 5 s (p99); station-health verdicts every 5 min.
- At-least-once delivery, idempotent consumers keyed by (robot_id, t).
- Raw logs kept 30 days for replay through the same detectors offline (the code in this repo).

## Open questions

- What can the robot send on a cellular budget? 1 Hz summaries plus events, with 10 Hz only around
  alarms, is the starting assumption.
- Who acts on a station flag: the fleet operator, the correction network, or both?
