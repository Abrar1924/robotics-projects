# ROS 2 checks

Saved output of the runs that passed is in [logs](logs). All of this is Gazebo and replay; no mower was built.

## C++ supervisor at five places in the Gazebo yard

`bash run_on_construct.sh` on ROS 2 Humble with Gazebo Classic. The mower is placed, not driven. GPS quality is scripted from the pose Gazebo reports. The check compares every answer of the C++ node with the Python supervisor.

```text
Gazebo moved the mower. Tree trunk at x=10 m, 1.5 m beside the lane. Wall face at x=19.9 m.
open grass: mode NOMINAL, speed 0.50
under a tree: mode DEGRADED, speed 0.32
under a tree, driver says GBAS_FIX and 3 cm, receiver says float: mode DEGRADED, speed 0.46
next to a wall: mode HOLD, speed 0.00
next to a wall, still bad after the wait: skip the strip
PASSED
```

The third line is the case `NavSatStatus` gets wrong by itself: it reports RTK fixed and RTK float with the same code. The node believes the receiver's quality code on `/gnss/rtk_quality` and slows down.

## The full loop with Nav2

`bash run_nav2.sh` on the same computer. Nav2's controller server follows the lane with Regulated Pure Pursuit. The mission script cancels the path on HOLD, and after the hold times out it tells the supervisor to retreat and sends a path back towards the last RTK-fixed spot.

```text
open grass: mode NOMINAL, speed limit 0.50, Nav2 command 0.50
under a tree: mode DEGRADED, speed limit 0.32, Nav2 command 0.32
next to a wall: mode HOLD, path cancelled, Nav2 command 0.00, mower moved 0.0 cm in 10 s
hold timed out: strip by the wall given up, supervisor told to retreat
retreat: mode RETREAT, speed limit 0.30, Nav2 command 0.30, driving away from the wall
back in open grass: mode NOMINAL at x = 18.9 m
closest the mower body came to the wall: 0.53 m
PASSED
```

## Python node

`docker build -f ros2/Dockerfile -t mower-supervisor . && docker run --rm mower-supervisor` on ROS 2 Jazzy, or `python3 -m mower_supervisor.check` with ROS 2 sourced.

```text
open grass: mode NOMINAL, speed 0.5
under a tree: mode DEGRADED, speed 0.32
next to a wall: mode HOLD, speed 0.0
next to a wall, still bad after the wait: skip the strip
PASSED
```

## C++ equal to Python, without ROS

`pytest tests/test_cpp_parity.py` compiles `ros2/mower_supervisor_cpp/src/supervisor.cpp` with g++ and compares it with the Python supervisor on the 1,653 readings of the real receiver log, at 5 m and at 0.30 m of clearance, and on 20,000 random readings that reach every mode and a timed-out hold.
