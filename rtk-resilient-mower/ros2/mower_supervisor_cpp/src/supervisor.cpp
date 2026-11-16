#include "supervisor.hpp"

#include <algorithm>

namespace mower {

double Supervisor::limit(double clearance) const {
  double raw = clearance / cfg.risk_k;
  raw = std::max(cfg.sigma_floor, raw);
  raw = std::min(cfg.sigma_max, raw);
  return raw;
}

Mode Supervisor::update(double t, int quality, double sigma, double clearance) {
  const double lim = limit(clearance);
  sigma_ = sigma;
  limit_ = lim;
  hold_expired = false;
  const bool good = quality == FIXED && sigma < cfg.sigma_nominal;
  Mode nxt = mode;
  if (mode == Mode::RETREAT) {
    nxt = good ? Mode::NOMINAL : Mode::RETREAT;
  } else if (mode == Mode::HOLD) {
    if (good || sigma < cfg.resume_ratio * lim) {
      nxt = good ? Mode::NOMINAL : Mode::DEGRADED;
    } else if (t - hold_since >= cfg.hold_timeout) {
      hold_expired = true;
      nxt = sigma >= cfg.sigma_max ? Mode::RETREAT : Mode::DEGRADED;
    } else {
      nxt = Mode::HOLD;
    }
  } else if (sigma >= lim) {
    nxt = Mode::HOLD;
    hold_since = t;
  } else {
    nxt = good ? Mode::NOMINAL : Mode::DEGRADED;
  }
  mode = nxt;
  return mode;
}

void Supervisor::force(Mode next) { mode = next; }

double Supervisor::speed_limit() const {
  if (mode == Mode::DEGRADED) {
    double headroom = 1.0 - sigma_ / limit_;
    headroom = std::min(std::max(headroom, 0.0), 1.0);
    return cfg.v_degraded + (cfg.v_nominal - cfg.v_degraded) * headroom;
  }
  if (mode == Mode::NOMINAL) return cfg.v_nominal;
  if (mode == Mode::HOLD) return 0.0;
  return cfg.v_retreat;
}

const char* mode_name(Mode mode) {
  switch (mode) {
    case Mode::NOMINAL:
      return "NOMINAL";
    case Mode::DEGRADED:
      return "DEGRADED";
    case Mode::HOLD:
      return "HOLD";
    case Mode::RETREAT:
      return "RETREAT";
  }
  return "NOMINAL";
}

int quality_from_status(int status) {
  if (status >= 2) return FIXED;
  if (status >= 0) return FLOAT_FIX;
  return 0;
}

int quality_from_gga(int gga) {
  if (gga == 4) return FIXED;
  if (gga == 5 || gga == 2) return FLOAT_FIX;
  return 0;
}

}  // namespace mower
