#pragma once

namespace mower {

constexpr int FIXED = 4;
constexpr int FLOAT_FIX = 5;

enum class Mode { NOMINAL, DEGRADED, HOLD, RETREAT };

struct Config {
  double v_nominal = 0.50;
  double v_degraded = 0.20;
  double v_retreat = 0.30;
  double sigma_nominal = 0.04;
  double sigma_floor = 0.05;
  double sigma_max = 0.25;
  double risk_k = 3.0;
  double resume_ratio = 0.6;
  double hold_timeout = 10.0;
};

class Supervisor {
 public:
  Config cfg;
  Mode mode = Mode::NOMINAL;
  double hold_since = 0.0;
  bool hold_expired = false;

  // Largest acceptable 1-sigma position error at this clearance to a hazard (m).
  double limit(double clearance) const;
  // One GNSS reading: time (s), quality (FIXED, FLOAT_FIX or 0), 1-sigma error (m), and the
  // distance to the nearest hazard (m). Returns the new mode.
  Mode update(double t, int quality, double sigma, double clearance);
  // The mission layer may overrule the mode, for example to send the mower back to its last
  // RTK-fixed spot after a hold has timed out.
  void force(Mode next);
  double speed_limit() const;

 private:
  double sigma_ = 0.0;
  double limit_ = 1.0;
};

const char* mode_name(Mode mode);
// sensor_msgs/NavSatStatus cannot say whether an RTK solution is fixed or float: drivers report
// both as GBAS_FIX. Use quality_from_gga when the receiver's own quality code is available.
int quality_from_status(int status);
// NMEA GGA quality indicator: 4 is RTK fixed, 5 is RTK float, 2 is differential.
int quality_from_gga(int gga);

}  // namespace mower
