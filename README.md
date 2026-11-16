# Robotics Projects: Positioning You Can Trust

Two projects about the same moment: a robot's RTK-GNSS position starts to slip, and the robot has to notice and degrade safely instead of failing. Both are simulations with C++ ROS 2 nodes, tested against their Python models.

| Project | What it does | Checked with |
|---|---|---|
| [gnss-integrity-monitor](gnss-integrity-monitor) | Detects GNSS spoofing, multipath and jamming on a drone with a GNSS/INS Kalman filter and three dual-antenna tests (0.20 m baseline), then cuts GNSS out of the loop | 21 tests, ROS 2 replay, PX4 v1.14.3 SITL |
| [rtk-resilient-mower](rtk-resilient-mower) | Mower that slows, stops, skips or retreats when its position uncertainty exceeds the room around it; site-readiness survey; real u-blox log replay | 25 tests, Gazebo, Nav2 |

Headline results:

- **Drone:** a 0.03 m/s² drift spoof hides from every inertial test (0/10 detected). The baseline-length test caught it on 10/10 flights in 0.2 s and cut the drag from 13.5 m to 0.6 m, with 0 false alarms in 2 h. The hardest attacker modelled (an attitude-aware array) is detected on 10/10 flights but not in time on 7 of them; the README says what would close that gap.
- **Mower:** 0 contact events in 10 missions against 2.3 per mission for EKF fusion alone; 89% mowed and 7% reported as deferred. Under Nav2 in Gazebo it makes a true stop beside a wall, then retreats and recovers.

Each folder has its own README with the full results, interfaces, limitations and the commands that reproduce them.

Mohammad Khan · MIT licensed
