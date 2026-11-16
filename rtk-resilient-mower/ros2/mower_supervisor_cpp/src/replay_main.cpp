// Replay GNSS readings through the supervisor. No ROS needed.
//
//   g++ -std=c++17 -O2 src/supervisor.cpp src/replay_main.cpp -o replay
//   ./replay < readings.txt
//
// One input line per reading:  t  quality  sigma  clearance
//   quality is the NMEA GGA quality indicator (4 RTK fixed, 5 RTK float, 2 differential)
//   clearance may be "inf"
// One output line per reading: mode  speed_limit  hold_expired
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "supervisor.hpp"

int main() {
  mower::Supervisor supervisor;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    double t = 0.0, sigma = 0.0;
    int gga = 0;
    std::string clearance_text;
    if (!(in >> t >> gga >> sigma >> clearance_text)) continue;
    const double clearance = std::stod(clearance_text);
    const mower::Mode mode = supervisor.update(t, mower::quality_from_gga(gga), sigma, clearance);
    std::printf("%s %.17g %d\n", mower::mode_name(mode), supervisor.speed_limit(),
                supervisor.hold_expired ? 1 : 0);
  }
  return 0;
}
