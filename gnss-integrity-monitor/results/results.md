### Detection (10 flights per scenario; detections within 60 s, median delay)

| Scenario | Snapshot χ² (one fix) | Sliding-window χ² (5 s) | Accelerometer-bias plausibility | Isolation Forest (learned) | Dual-antenna baseline length | Dual-antenna heading vs gyro | Course vs heading (3 s) |
|---|---|---|---|---|---|---|---|
| Nominal | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms |
| Multipath step (1.5 m, 10 s) | 10/10, 0.0 s | 10/10, 0.0 s | 10/10, 0.1 s | 10/10, 0.2 s | 10/10, 0.2 s | 10/10, 0.2 s | 10/10, 10.8 s |
| Honest RTK float (receiver reports 0.3 m, 30 s) | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms | 0/10 false alarms |
| Jamming (20 s outage) | n/a (no fixes to test) | n/a (no fixes to test) | n/a (no fixes to test) | n/a (no fixes to test) | n/a (no fixes to test) | n/a (no fixes to test) | n/a (no fixes to test) |
| Crude spoof (8 m jump) | 10/10, 0.0 s | 10/10, 0.0 s | 10/10, 0.0 s | 10/10, 0.2 s | 10/10, 0.2 s | 10/10, 0.2 s | 0/10 |
| Drift spoof, aggressive (0.2 m/s²) | 10/10, 1.6 s | 10/10, 1.5 s | 10/10, 2.9 s | 10/10, 2.1 s | 10/10, 0.2 s | 10/10, 0.2 s | 0/10 |
| Drift spoof, moderate (0.1 m/s²) | 2/10, 22.4 s | 10/10, 2.6 s | 3/10, 4.5 s | 9/10, 3.2 s | 10/10, 0.2 s | 10/10, 0.2 s | 0/10 |
| Drift spoof, gentle (0.03 m/s²) | 0/10 | 0/10 | 0/10 | 0/10 | 10/10, 0.2 s | 10/10, 0.2 s | 0/10 |
| Gentle drift spoof from an array replaying a frozen baseline | 0/10 | 0/10 | 0/10 | 0/10 | 0/10 | 10/10, 2.2 s | 10/10, 2.6 s |
| Gentle drift spoof from an attitude-aware array | 0/10 | 0/10 | 0/10 | 0/10 | 0/10 | 0/10 | 10/10, 17.3 s |

### Outcome with the monitor in the loop (median of 10 flights)

| Scenario | Error without monitor (30 s after onset) | Error with monitor, IMU tests (until alarm + 10 s) | Error with monitor + dual antenna | Flights dragged past the 5 m geofence margin: none → IMU → dual |
|---|---|---|---|---|
| Nominal | 0.0 m | 0.0 m | 0.0 m | 0/10 → 0/10 → 0/10 |
| Multipath step (1.5 m, 10 s) | 2.0 m | 1.6 m | 0.7 m | 0/10 → 0/10 → 0/10 |
| Honest RTK float (receiver reports 0.3 m, 30 s) | 0.3 m | 0.3 m | 0.3 m | 0/10 → 0/10 → 0/10 |
| Jamming (20 s outage) | 3.5 m | 3.5 m | 3.5 m | 1/10 → 1/10 → 1/10 |
| Crude spoof (8 m jump) | 10.5 m | 1.0 m | 0.7 m | 10/10 → 0/10 → 0/10 |
| Drift spoof, aggressive (0.2 m/s²) | 50.0 m | 1.0 m | 0.6 m | 10/10 → 0/10 → 0/10 |
| Drift spoof, moderate (0.1 m/s²) | 40.0 m | 1.3 m | 0.6 m | 10/10 → 0/10 → 0/10 |
| Drift spoof, gentle (0.03 m/s²) | 13.5 m | 13.5 m | 0.6 m | 10/10 → 10/10 → 0/10 |
| Gentle drift spoof from an array replaying a frozen baseline | 13.5 m | 13.5 m | 0.9 m | 10/10 → 10/10 → 0/10 |
| Gentle drift spoof from an attitude-aware array | 13.5 m | 13.5 m | 6.3 m | 10/10 → 10/10 → 7/10 |

### False alarms per hour (2.0 h of unseen clean and honestly degraded flight)

| Snapshot χ² (one fix) | Sliding-window χ² (5 s) | Accelerometer-bias plausibility | Isolation Forest (learned) | Dual-antenna baseline length | Dual-antenna heading vs gyro | Course vs heading (3 s) | Any classical test | Any test, with dual antenna |
|---|---|---|---|---|---|---|---|---|
| 0.50 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.50 | 0.50 |

### Drift-spoofing sweep, single transmit antenna (share of flights detected within 60 s)

| Fake acceleration (m/s²) | Snapshot χ² (one fix) | Sliding-window χ² (5 s) | Accelerometer-bias plausibility | Isolation Forest (learned) | Dual-antenna baseline length |
|---|---|---|---|---|---|
| 0.01 | 0% | 0% | 0% | 0% | 100% |
| 0.02 | 0% | 0% | 0% | 0% | 100% |
| 0.03 | 0% | 0% | 0% | 0% | 100% |
| 0.05 | 0% | 0% | 0% | 0% | 100% |
| 0.1 | 20% | 100% | 30% | 90% | 100% |
| 0.2 | 100% | 100% | 100% | 100% | 100% |
| 0.5 | 100% | 100% | 100% | 100% | 100% |

### Spoofers that fake the antenna baseline (onset spread evenly over one lap)

| Scenario | Test that catches it | Detected | Delay: median (range) | Drag at the alarm (median) |
|---|---|---|---|---|
| Gentle drift spoof from an array replaying a frozen baseline | Dual-antenna heading vs gyro | 10/10 | 2.2 s (0.5–5.9 s) | 0.1 m |
| Gentle drift spoof from an attitude-aware array | Course vs heading (3 s) | 10/10 | 17.3 s (11.3–25.4 s) | 4.5 m |
