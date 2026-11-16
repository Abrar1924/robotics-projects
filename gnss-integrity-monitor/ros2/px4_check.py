"""Feed PX4 a GPS fix and check the C++ monitor against the course and yaw PX4 reports.

PX4 v1.14 is the flight program. This script is the simulator on the other end of the link
PX4 already uses (TCP port 4560). The IMU keeps the airframe level and pointing whichever way
the estimator settles. The GPS course is sent equal to that yaw, then 0.7 rad away from it.

Everything the monitor sees comes back from PX4 over MAVLink, not from the numbers this script
sent: velocity from GPS_RAW_INT, yaw and yaw rate from ATTITUDE. PX4 works in north-east-down
and the monitor in east-north-up, so the script converts on the way.

PX4 SITL has one GPS antenna, so there is no moving-base solution to read. The antenna baseline
is built from PX4's own yaw: 0.20 m long, pointing along the nose. That makes this a check of
the course test and of the frame conversions on real PX4 telemetry. It does not exercise the
baseline-length test, and it is not a flight.

The C++ monitor has to keep GNSS trusted for 5 s while course and yaw agree, and raise a
course alarm once they do not.
"""
from __future__ import annotations

import inspect
import math
import os
import socket
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3Stamped
from pymavlink import mavutil
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

TCP_PORT = 4560
LIE_OFFSET = 0.7
SPEED = 3.0
BASELINE = 0.20
# Velocity variance passed to the C++ monitor, (m/s)^2 per axis. At 3 m/s that is about
# 2 degrees of course uncertainty: estimator noise stays under the alarm line, 0.7 rad does not.
VELOCITY_VAR = 0.01
CLEAN_EPOCHS = 50      # 5 s of trusted GNSS at 10 Hz before the course is pulled away
SETTLE_SAMPLES = 10    # GPS samples in a row that must match the yaw before the monitor is fed
HOME_LAT = 37.4
HOME_LON = -122.0
# Body-frame field for a level vehicle at this home, yaw near north.
MAG = (0.234, 0.054, 0.416)
FIELDS_ALL = 0x1FFF
QOS = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def course_bits(course: float) -> tuple[int, int, int, int]:
    """vn, ve, speed, cog in the units HIL_GPS wants."""
    vn = int(round(SPEED * math.cos(course) * 100.0))
    ve = int(round(SPEED * math.sin(course) * 100.0))
    vel = int(round(math.hypot(vn, ve)))
    cog = int(round(math.degrees(course) * 100.0)) % 36000
    return vn, ve, vel, cog


def stamp(seconds: float) -> Time:
    sec = int(seconds)
    return Time(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class Check(Node):
    """Publishes PX4's telemetry on the monitor's topics (ENU) and collects its answers."""

    def __init__(self):
        super().__init__("px4_monitor_check")
        self.imu_pub = self.create_publisher(Imu, "/gim/imu", QOS)
        self.baseline_pub = self.create_publisher(Vector3Stamped, "/gim/baseline", QOS)
        self.velocity_pub = self.create_publisher(TwistWithCovarianceStamped, "/gim/velocity", QOS)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diag, QOS)
        self.answers = []   # (trusted, alarm source, course statistic)

    def _on_diag(self, msg):
        if not msg.status:
            return
        status = msg.status[0]
        values = {kv.key: kv.value for kv in status.values}
        self.answers.append((status.level == DiagnosticStatus.OK, values.get("alarm_source", ""),
                             float(values.get("course_stat", "nan"))))

    def connected(self) -> bool:
        return (self.imu_pub.get_subscription_count() > 0 and self.baseline_pub.get_subscription_count() > 0
                and self.velocity_pub.get_subscription_count() > 0 and self.count_publishers("/diagnostics") > 0)

    def publish_gyro(self, t: float, yawspeed_ned: float) -> None:
        msg = Imu()
        msg.header.stamp = stamp(t)
        msg.angular_velocity.z = -yawspeed_ned      # NED yaw rate is clockwise, ENU counter-clockwise
        self.imu_pub.publish(msg)

    def publish_epoch(self, t: float, speed: float, course_ned: float, yaw_ned: float) -> None:
        when = stamp(t)
        b = Vector3Stamped()
        b.header.stamp = when
        b.header.frame_id = "map"
        b.vector.x = BASELINE * math.sin(yaw_ned)   # east
        b.vector.y = BASELINE * math.cos(yaw_ned)   # north
        self.baseline_pub.publish(b)
        v = TwistWithCovarianceStamped()
        v.header.stamp = when
        v.header.frame_id = "map"
        v.twist.twist.linear.x = speed * math.sin(course_ned)   # east
        v.twist.twist.linear.y = speed * math.cos(course_ned)   # north
        v.twist.covariance[0] = VELOCITY_VAR
        v.twist.covariance[7] = VELOCITY_VAR
        self.velocity_pub.publish(v)


class Sitl:
    def __init__(self, conn: socket.socket):
        self.conn = conn
        self.conn.settimeout(0.05)
        self.mav = mavutil.mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        self.time_us = 1_000_000
        self.gps_div = 0
        self.north = 0.0
        self.east = 0.0
        self.course = 0.0
        self.sensor_encode = self._pick_encode("hil_sensor_encode")
        self.gps_encode = self._pick_encode("hil_gps_encode")

    def _pick_encode(self, name: str):
        fn = getattr(self.mav, name)
        return fn, list(inspect.signature(fn).parameters)

    def _send(self, msg) -> None:
        self.conn.sendall(msg.pack(self.mav))

    def _call(self, spec, args: dict):
        fn, names = spec
        # pymavlink puts the MAVLink instance first on unbound encode methods.
        names = [n for n in names if n not in ("self", "mav")]
        return fn(*[args[n] for n in names if n in args])

    def step(self) -> None:
        self.time_us += 4000
        self._send(self._call(self.sensor_encode, {
            "time_usec": self.time_us,
            "xacc": 0.0,
            "yacc": 0.0,
            "zacc": -9.80665,
            "xgyro": 0.0,
            "ygyro": 0.0,
            "zgyro": 0.0,
            "xmag": MAG[0],
            "ymag": MAG[1],
            "zmag": MAG[2],
            "abs_pressure": 1013.25,
            "diff_pressure": 0.0,
            "pressure_alt": 0.0,
            "temperature": 25.0,
            "fields_updated": FIELDS_ALL,
            "id": 0,
        }))
        self.gps_div += 1
        if self.gps_div < 25:
            return
        self.gps_div = 0
        dt = 0.1
        self.north += SPEED * math.cos(self.course) * dt
        self.east += SPEED * math.sin(self.course) * dt
        lat = HOME_LAT + self.north / 111111.0
        lon = HOME_LON + self.east / (111111.0 * math.cos(math.radians(HOME_LAT)))
        vn, ve, vel, cog = course_bits(self.course)
        self._send(self._call(self.gps_encode, {
            "time_usec": self.time_us,
            "fix_type": 3,
            "lat": int(lat * 1e7),
            "lon": int(lon * 1e7),
            "alt": 100_000,
            "eph": 100,
            "epv": 100,
            "vel": vel,
            "vn": vn,
            "ve": ve,
            "vd": 0,
            "cog": cog,
            "satellites_visible": 12,
            "id": 0,
            "yaw": 65535,
        }))

    def drain(self) -> bool:
        try:
            data = self.conn.recv(16384)
        except socket.timeout:
            return False
        if not data:
            raise ConnectionError("PX4 closed the simulator connection")
        return True


def start_px4(root: str, log) -> subprocess.Popen:
    build = os.path.join(root, "build", "px4_sitl_default")
    binary = os.path.join(build, "bin", "px4")
    env = os.environ.copy()
    env["PX4_SIM_MODEL"] = "none_iris"
    env.pop("PX4_SIMULATOR", None)
    env.pop("PX4_GZ_MODEL", None)
    return subprocess.Popen(
        [binary, "-d"],
        cwd=build,
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def request_streams(link) -> None:
    for msg_id, usec in (
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 100000),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20000),
    ):
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            usec,
            0, 0, 0, 0, 0,
        )


def gps_sample(msg) -> tuple[float, float, float] | None:
    """Time (s since PX4 boot), speed (m/s) and course (rad from north, clockwise) of one GPS_RAW_INT."""
    if msg.vel == 65535 or msg.cog == 65535:
        return None
    speed = msg.vel / 100.0
    if speed < 1.0:
        return None
    course = wrap(math.radians(msg.cog / 100.0))
    return msg.time_usec * 1e-6, speed, course


def main() -> int:
    root = os.environ.get("PX4_ROOT", os.path.expanduser("~/PX4-Autopilot"))
    binary = os.path.join(root, "build", "px4_sitl_default", "bin", "px4")
    if not os.path.isfile(binary):
        print("FAILED")
        print(f"PX4 is not built at {binary}")
        return 1

    rclpy.init()
    node = Check()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + 20.0
    while not node.connected() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    if not node.connected():
        print("FAILED")
        print("The C++ monitor is not running.")
        rclpy.shutdown()
        return 1

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", TCP_PORT))
    server.listen(1)
    server.settimeout(120.0)

    log = open("/tmp/px4.log", "w")
    proc = start_px4(root, log)
    link = None
    try:
        try:
            conn, _ = server.accept()
        except socket.timeout:
            print("FAILED")
            print("PX4 never connected to the simulator. Last lines of /tmp/px4.log:")
            log.flush()
            tail_log()
            return 1
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sitl = Sitl(conn)
        print("PX4 connected. Waiting for its own GPS and yaw.")

        link = mavutil.mavlink_connection("udpin:127.0.0.1:14540", source_system=255)
        # An empty read must return at once. A blocking read stalls PX4's clock.
        link.port.settimeout(0.0)
        streams = False
        yaw = None
        got = None
        settled = 0
        clean_yaw = None
        lie_course = None
        phase = "settle"
        t0 = time.monotonic()
        last_report = t0
        phase_started = t0
        kicked = 0.0

        while True:
            if proc.poll() is not None:
                print("FAILED")
                print("PX4 stopped. Last lines of /tmp/px4.log:")
                log.flush()
                tail_log()
                return 1
            got_packet = sitl.drain()
            now = time.monotonic()
            if got_packet:
                sitl.step()
            elif now - kicked > 0.3:
                # The first heartbeat is the only packet until sensor time starts.
                sitl.step()
                kicked = now
            executor.spin_once(timeout_sec=0.0)

            fresh = False
            msg = link.recv_match(blocking=False)
            if msg is not None and msg.get_type() == "HEARTBEAT" and not streams:
                request_streams(link)
                streams = True
                print("PX4 heartbeat received.")
            elif msg is not None and msg.get_type() == "ATTITUDE":
                if math.isfinite(msg.yaw):
                    yaw = float(msg.yaw)
                    node.publish_gyro(msg.time_boot_ms * 1e-3, float(msg.yawspeed))
            elif msg is not None and msg.get_type() == "GPS_RAW_INT":
                sample = gps_sample(msg)
                if sample is not None:
                    got = sample
                    fresh = True

            if now - last_report > 5.0:
                last_report = now
                course_s = f"{got[2]:.2f}" if got else "none"
                yaw_s = f"{yaw:.2f}" if yaw is not None else "none"
                print(f"still waiting ({phase}): gps course {course_s} rad, yaw {yaw_s} rad")

            if not fresh or yaw is None or got is None:
                if now - phase_started > 180.0:
                    print("FAILED")
                    print(f"No usable GPS and yaw from PX4 while waiting ({phase}).")
                    return 1
                continue
            t_gps, speed, course = got

            if phase == "settle":
                # Keep the GPS course on the airframe yaw until PX4 reports the two together.
                sitl.course = yaw
                settled = settled + 1 if abs(wrap(course - yaw)) < 0.05 else 0
                if settled >= SETTLE_SAMPLES:
                    phase, phase_started = "clean", now
                    node.answers.clear()
                elif now - phase_started > 180.0:
                    print("FAILED")
                    print("PX4 did not publish a GPS course that matches its yaw.")
                    return 1
            elif phase == "clean":
                sitl.course = yaw
                node.publish_epoch(t_gps, speed, course, yaw)
                if any(not trusted for trusted, _source, _stat in node.answers):
                    source = next(src for trusted, src, _stat in node.answers if not trusted)
                    print("FAILED")
                    print(f"False alarm from the {source} test while GPS course and yaw agreed.")
                    return 1
                if len(node.answers) >= CLEAN_EPOCHS and math.isfinite(node.answers[-1][2]):
                    clean_yaw = yaw
                    lie_course = wrap(clean_yaw + LIE_OFFSET)
                    sitl.course = lie_course
                    print(f"PX4 clean GPS: course {course:.2f} rad, yaw {yaw:.2f} rad, "
                          f"{len(node.answers)} epochs, course statistic {node.answers[-1][2]:.2f} "
                          "(alarm above 19.5), GNSS trusted")
                    phase, phase_started = "lie", now
                elif now - phase_started > 120.0:
                    print("FAILED")
                    print("The C++ monitor did not answer on PX4's clean GPS.")
                    return 1
            else:
                node.publish_epoch(t_gps, speed, course, yaw)
                alarm = next(((src, stat) for trusted, src, stat in node.answers if not trusted), None)
                if alarm is not None:
                    source, stat = alarm
                    if source == "course" and abs(wrap(course - yaw)) > 0.4:
                        print(f"PX4 GPS course pulled off the airframe: course {course:.2f} rad, "
                              f"yaw {yaw:.2f} rad, course statistic {stat:.1f}, alarm from the course test")
                        print("PASSED")
                        return 0
                    print("FAILED")
                    print(f"Alarm from the {source} test at course {course:.2f} rad, yaw {yaw:.2f} rad.")
                    return 1
                if now - phase_started > 90.0:
                    print("FAILED")
                    print("The lying GPS course did not make the C++ monitor alarm.")
                    return 1
    finally:
        if link is not None:
            link.close()
        server.close()
        log.close()
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, 15)
            except OSError:
                proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, 9)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def tail_log() -> None:
    path = "/tmp/px4.log"
    if not os.path.isfile(path):
        return
    with open(path) as handle:
        lines = handle.readlines()[-30:]
    for line in lines:
        print(line.rstrip())


if __name__ == "__main__":
    sys.exit(main())
