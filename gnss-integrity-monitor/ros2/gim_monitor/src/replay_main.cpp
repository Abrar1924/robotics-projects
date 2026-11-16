// Replay a recorded flight through the dual-antenna monitor. No ROS needed.
//
//   g++ -std=c++17 -O2 -Iinclude src/replay_main.cpp -o replay
//   ./replay < ../replay/attitude_aware.txt            # every epoch judged on its own
//   ./replay --latch < ../replay/attitude_aware.txt    # flight behaviour: alarms latch
//
// Input lines are described in gim/export.py. One output line per GNSS epoch:
//   t baseline_stat heading_nis course_residual course_stat baseline heading course trusted source
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include "gim_monitor/dual_antenna.hpp"

int main(int argc, char** argv) {
  bool latch = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--latch") == 0) latch = true;
  }
  gim::Monitor monitor(gim::kBaseline, gim::kSigmaSlip, latch);
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    char kind = 0;
    in >> kind;
    if (kind == 'H') {
      double heading0 = 0.0;
      in >> heading0;
      monitor.init(heading0);
    } else if (kind == 'G') {
      double t = 0.0, rate = 0.0, dt = 0.0;
      in >> t >> rate >> dt;
      monitor.gyro(rate, dt);
    } else if (kind == 'E') {
      double t = 0.0, bx = 0.0, by = 0.0, sigma_b = 0.0, vx = 0.0, vy = 0.0;
      double c00 = 0.0, c01 = 0.0, c11 = 0.0, drift = 0.0;
      int has_b = 0;
      in >> t >> has_b >> bx >> by >> sigma_b >> vx >> vy >> c00 >> c01 >> c11 >> drift;
      const gim::Epoch e = monitor.epoch(has_b != 0, bx, by, sigma_b, vx, vy, c00, c01, c11, drift);
      std::printf("%.9g %.17g %.17g %.17g %.17g %d %d %d %d %s\n", t, e.baseline_stat, e.heading_nis,
                  e.course_residual, e.course_stat, e.baseline_alarm ? 1 : 0, e.heading_alarm ? 1 : 0,
                  e.course_alarm ? 1 : 0, e.trusted ? 1 : 0,
                  e.source == gim::Source::NONE ? "-" : gim::source_name(e.source));
    }
  }
  return 0;
}
