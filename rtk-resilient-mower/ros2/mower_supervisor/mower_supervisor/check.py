"""Publish three GPS situations and require the supervisor's answers.

Runs in one process with real ROS 2 topics. The readings are made up so the
three answers can be checked. This does not draw a yard and does not use a
recording.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from mower_supervisor.node import MowerSupervisor
from mower.show_cases import situations


def _stamp(msg, t):
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))


class Probe(Node):
    def __init__(self):
        super().__init__("mower_probe")
        self.clearance_pub = self.create_publisher(Float64, "/hazard_clearance", 10)
        self.fix_pub = self.create_publisher(NavSatFix, "/gnss/fix", 10)
        self.create_subscription(String, "/mower/mode", self._on_mode, 10)
        self.create_subscription(Float64, "/mower/speed_limit", self._on_speed, 10)
        self.create_subscription(String, "/mower/event", self._on_event, 10)
        self.mode = None
        self.speed = None
        self.events = []

    def _on_mode(self, msg):
        self.mode = msg.data

    def _on_speed(self, msg):
        self.speed = msg.data

    def _on_event(self, msg):
        self.events.append(msg.data)

    def publish_clearance(self, meters):
        msg = Float64()
        msg.data = float(meters)
        self.clearance_pub.publish(msg)

    def publish_fix(self, status, sigma_m, t):
        self.mode = None
        self.speed = None
        fix = NavSatFix()
        _stamp(fix, t)
        fix.status.status = int(status)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.position_covariance[0] = float(sigma_m) ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_pub.publish(fix)


def _spin(executor, times=8):
    for _ in range(times):
        executor.spin_once(timeout_sec=0.1)


def main():
    rclpy.init()
    supervisor = MowerSupervisor()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(supervisor)
    executor.add_node(probe)
    failures = []
    lines = []

    def play(case, t):
        probe.publish_clearance(case["clearance_m"])
        _spin(executor)
        probe.publish_fix(case["status"], case["sigma_m"], t)
        _spin(executor)
        return probe.mode, probe.speed

    for i, case in enumerate(situations()):
        mode, speed = play(case, float(i))
        expected = case["mode"].name
        ok = mode == expected
        if "speed_mps" in case and speed is not None:
            ok = ok and abs(speed - case["speed_mps"]) < 1e-6
        if case["name"] == "under a tree":
            ok = ok and speed is not None and 0.20 <= speed < 0.50
        line = f"{case['name']}: mode {mode}, speed {speed}"
        lines.append(line)
        print(line)
        if not ok:
            failures.append(f"{line} (wanted {expected})")

    wall = situations()[-1]
    play(wall, 30.0)
    if "skip" in probe.events:
        print("next to a wall, still bad after the wait: skip the strip")
    else:
        failures.append("beside the wall for 11 s: no skip")

    supervisor.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()
    if failures:
        print("FAILED")
        for item in failures:
            print(item)
        sys.exit(1)
    print("PASSED")


if __name__ == "__main__":
    main()
