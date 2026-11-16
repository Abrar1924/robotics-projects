"""Replay four recorded flights through the C++ monitor node and compare its decisions with Python's.

The flights are simulations (ros2/replay/*.txt, written by `python -m gim.export`): one clean
flight and one per spoofer geometry. expected.json holds what the Python tests decided on the
same inputs. Each flight goes to a fresh node over its real topics, one GNSS epoch at a time,
and the node has to raise the same alarm at the same moment.

This is a replay of simulated sensor data. It is not a flight and not a real antenna recording.
"""
from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

REPLAY = Path(__file__).resolve().parent / "replay"
QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
TOLERANCE = 0.15   # s: the node must alarm within one or two epochs of the Python tests


def stamp(t: float) -> Time:
    sec = int(t)
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)))


class Replayer(Node):
    def __init__(self):
        super().__init__("gim_replayer")
        self.imu_pub = self.create_publisher(Imu, "/gim/imu", QOS)
        self.baseline_pub = self.create_publisher(Vector3Stamped, "/gim/baseline", QOS)
        self.velocity_pub = self.create_publisher(TwistWithCovarianceStamped, "/gim/velocity", QOS)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, QOS)
        self.answers = {}   # epoch stamp in ns -> (trusted, alarm source)

    def _on_diag(self, msg):
        if not msg.status:
            return
        status = msg.status[0]
        values = {kv.key: kv.value for kv in status.values}
        key = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.answers[key] = (status.level == DiagnosticStatus.OK, values.get("alarm_source", ""))

    def connected(self):
        return (self.imu_pub.get_subscription_count() > 0 and self.baseline_pub.get_subscription_count() > 0
                and self.velocity_pub.get_subscription_count() > 0 and self.count_publishers("/diagnostics") > 0)


def replay(node, executor, path):
    """Send one flight. Returns [(t, trusted, source)] for every epoch the node answered."""
    out = []
    node.answers.clear()
    for line in path.read_text().splitlines():
        parts = line.split()
        if parts[0] == "G":
            msg = Imu()
            msg.header.stamp = stamp(float(parts[1]))
            msg.angular_velocity.z = float(parts[2])
            node.imu_pub.publish(msg)
        elif parts[0] == "E":
            t = float(parts[1])
            when = stamp(t)
            has_baseline = parts[2] == "1"
            if has_baseline:
                b = Vector3Stamped()
                b.header.stamp = when
                b.header.frame_id = "map"
                b.vector.x, b.vector.y = float(parts[3]), float(parts[4])
                node.baseline_pub.publish(b)
            v = TwistWithCovarianceStamped()
            v.header.stamp = when
            v.header.frame_id = "map"
            v.twist.twist.linear.x, v.twist.twist.linear.y = float(parts[6]), float(parts[7])
            v.twist.covariance[0], v.twist.covariance[1] = float(parts[8]), float(parts[9])
            v.twist.covariance[6], v.twist.covariance[7] = float(parts[9]), float(parts[10])
            node.velocity_pub.publish(v)
            if not has_baseline:
                continue   # the node answers this epoch when the next one arrives
            key = when.sec * 1_000_000_000 + when.nanosec
            end = time.monotonic() + 5.0
            while key not in node.answers and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.01)
            if key not in node.answers:
                raise TimeoutError(f"no answer from the C++ monitor for the epoch at {t:.1f} s")
            out.append((t, *node.answers[key]))
    return out


def run_case(slug, want):
    node = Replayer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.1)
    if node.velocity_pub.get_subscription_count() > 0:
        executor.remove_node(node)
        node.destroy_node()
        return f"{slug}: another gim_monitor is already running. Stop it and run the check again."
    proc = subprocess.Popen(["ros2", "run", "gim_monitor", "monitor"], start_new_session=True,
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    try:
        end = time.monotonic() + 30.0
        while not node.connected() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
        if not node.connected():
            return f"{slug}: the C++ monitor did not start"
        answers = replay(node, executor, REPLAY / f"{slug}.txt")
    except TimeoutError as err:
        return f"{slug}: {err}"
    finally:
        executor.remove_node(node)
        node.destroy_node()
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except OSError:
                pass

    first = next(((t, source) for t, trusted, source in answers if not trusted), None)
    if len(answers) != want["epochs"]:
        return f"{slug}: the node answered {len(answers)} of {want['epochs']} epochs"
    if not want["alarm_source"]:
        if first is not None:
            return f"{slug}: false alarm from the {first[1]} test at {first[0]:.1f} s"
        print(f"{slug}: {len(answers)} epochs, GNSS trusted throughout (Python: no alarm)")
        return None
    if first is None:
        return f"{slug}: no alarm (Python: {want['alarm_source']} at {want['alarm_time']} s)"
    t, source = first
    line = (f"{slug}: {source} alarm at {t:.1f} s, {t - want['attack_start']:.1f} s after the attack began "
            f"(Python: {want['alarm_source']} at {want['alarm_time']:.1f} s)")
    if source != want["alarm_source"] or abs(t - want["alarm_time"]) > TOLERANCE:
        return line + " -- MISMATCH"
    if any(trusted for at, trusted, _ in answers if at > t):
        return line + " -- GNSS was trusted again after the alarm"
    print(line)
    return None


def main():
    expected = json.loads((REPLAY / "expected.json").read_text())
    rclpy.init()
    failures = [msg for slug, want in expected.items() if (msg := run_case(slug, want))]
    rclpy.shutdown()
    if failures:
        print("FAILED")
        for item in failures:
            print(item)
        sys.exit(1)
    print("PASSED")


if __name__ == "__main__":
    main()
