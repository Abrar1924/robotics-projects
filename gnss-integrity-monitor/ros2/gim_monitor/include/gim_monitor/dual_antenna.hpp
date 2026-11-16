// Dual-antenna GNSS integrity tests. Same model and numbers as gim/dual_antenna.py.
// No ROS in this file: it builds with any C++17 compiler, so it can run on a companion
// computer, inside a flight-stack module, or in a unit test.
//
//   baseline  measured antenna separation differs from the known length
//             -> spoofer with one transmit antenna
//   heading   antenna heading disagrees with the gyro-propagated heading
//             -> array replaying a frozen baseline
//   course    direction of travel disagrees with where the nose points (3 s mean)
//             -> attitude-aware array dragging the position sideways
//
// Angles are measured from the x axis towards the y axis (east towards north in ENU).
#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace gim {

constexpr double kPi = 3.14159265358979323846;
constexpr double kBaseline = 0.20;                 // m: antenna separation
constexpr double kSigmaB = 0.005;                  // m: 1-sigma per axis of a fixed moving-base solution
constexpr double kThreshold = 19.511420964666268;  // chi-square(1) at a 1e-5 false-alarm probability
constexpr int kPersist = 3;                        // epochs over threshold before an alarm (0.3 s at 10 Hz)
constexpr double kSigmaSlip = kPi / 180.0;         // rad: budget for the drift angle in nose-first flight
constexpr std::size_t kCourseWindow = 30;          // epochs averaged by the course test (3 s at 10 Hz)
constexpr std::size_t kCourseMin = 20;             // valid epochs the window needs before it may alarm
constexpr double kMinSpeed = 1.0;                  // m/s: course is undefined when nearly stationary
constexpr int kReacquire = 30;                     // rejected headings in a row before re-initialising
const double kNaN = std::numeric_limits<double>::quiet_NaN();

// Wrap an angle to [-pi, pi).
inline double wrap(double angle) {
  const double tau = 2.0 * kPi;
  double w = std::fmod(angle + kPi, tau);
  if (w < 0.0) w += tau;
  return w - kPi;
}

// Squared, normalised error of the measured baseline length: chi-square(1) when healthy.
inline double baseline_stat(double bx, double by, double sigma, double length = kBaseline) {
  const double e = (std::hypot(bx, by) - length) / sigma;
  return e * e;
}

// Two-state Kalman filter [heading, gyro bias], driven by the gyro, corrected by GNSS heading.
class HeadingFilter {
 public:
  explicit HeadingFilter(double heading0 = 0.0, double heading_sigma = 0.05, double bias_sigma = 0.003)
      : heading_(heading0), p00_(heading_sigma * heading_sigma), p11_(bias_sigma * bias_sigma) {}

  double heading() const { return heading_; }
  double gyro_bias() const { return bias_; }
  double heading_variance() const { return p00_; }

  void predict(double rate_meas, double dt) {
    heading_ = wrap(heading_ + (rate_meas - bias_) * dt);
    // P = F P F' + Q with F = [[1, -dt], [0, 1]]
    const double p00 = p00_ - 2.0 * dt * p01_ + dt * dt * p11_;
    const double p01 = p01_ - dt * p11_;
    p00_ = p00 + (2.0 * kRateNoise * dt) * (2.0 * kRateNoise * dt);
    p01_ = p01;
    p11_ = p11_ + kBiasWalk * kBiasWalk * dt;
  }

  struct Result {
    double baseline_stat;
    double heading_nis;
  };

  // Test one baseline fix and fuse its heading if it passes both tests.
  // reacquire = false keeps the filter on the gyro however long fixes are rejected.
  Result step(double bx, double by, double sigma_b, double length = kBaseline, bool reacquire = true) {
    const double len_stat = baseline_stat(bx, by, sigma_b, length);
    const double sigma_h = sigma_b / length;
    const double measured = std::atan2(by, bx);
    const double nu = wrap(measured - heading_);
    const double s = p00_ + sigma_h * sigma_h;
    const double nis = nu * nu / s;
    if (len_stat < kThreshold && nis < kThreshold) {
      const double k0 = p00_ / s;
      const double k1 = p01_ / s;
      heading_ = wrap(heading_ + k0 * nu);
      bias_ += k1 * nu;
      // Joseph form: P = (I - K H) P (I - K H)' + R K K', H = [1, 0]
      const double a00 = (1.0 - k0) * p00_;
      const double a01 = (1.0 - k0) * p01_;
      const double a10 = p01_ - k1 * p00_;
      const double a11 = p11_ - k1 * p01_;
      const double r = sigma_h * sigma_h;
      p00_ = a00 * (1.0 - k0) + r * k0 * k0;
      p01_ = a01 - a00 * k1 + r * k0 * k1;
      p11_ = a11 - a10 * k1 + r * k1 * k1;
      rejected_ = 0;
    } else if (len_stat < kThreshold && reacquire) {
      if (++rejected_ >= kReacquire) {
        heading_ = measured;
        p00_ = std::fmax(p00_, 4.0 * sigma_h * sigma_h);
        rejected_ = 0;
      }
    }
    return {len_stat, nis};
  }

 private:
  static constexpr double kRateNoise = 0.001;
  static constexpr double kBiasWalk = 1e-4;
  double heading_;
  double bias_ = 0.0;
  double p00_;
  double p01_ = 0.0;
  double p11_;
  int rejected_ = 0;
};

// Windowed test that the direction of travel agrees with where the nose points.
class CourseCheck {
 public:
  explicit CourseCheck(double sigma_slip = kSigmaSlip) : sigma_slip_(sigma_slip) {}

  struct Result {
    double residual;
    double stat;
  };

  double mean() const { return mean_; }

  // heading is NaN when the antenna heading is unusable.
  Result step(double vx, double vy, double c00, double c01, double c11, double heading, double sigma_h,
              double expected_drift) {
    const double speed = std::hypot(vx, vy);
    if (speed < kMinSpeed || !std::isfinite(heading)) {
      push(kNaN);
      return {kNaN, kNaN};
    }
    const double course = std::atan2(vy, vx);
    const double residual = wrap(course - heading + expected_drift);
    push(residual);
    double sum = 0.0;
    std::size_t n = 0;
    for (const double r : residuals_) {
      if (std::isfinite(r)) {
        sum += r;
        ++n;
      }
    }
    if (n < kCourseMin) return {residual, kNaN};
    const double px = -vy / speed;
    const double py = vx / speed;
    const double var_course = (px * px * c00 + py * py * c11 + 2.0 * px * py * c01) / (speed * speed);
    const double var = sigma_h * sigma_h / static_cast<double>(n) + sigma_slip_ * sigma_slip_ + var_course;
    mean_ = sum / static_cast<double>(n);
    return {residual, mean_ * mean_ / var};
  }

 private:
  void push(double value) {
    residuals_.push_back(value);
    if (residuals_.size() > kCourseWindow) residuals_.pop_front();
  }

  std::deque<double> residuals_;
  double sigma_slip_;
  double mean_ = 0.0;
};

enum class Source { NONE, BASELINE, HEADING, COURSE };

inline const char* source_name(Source s) {
  switch (s) {
    case Source::BASELINE:
      return "baseline";
    case Source::HEADING:
      return "heading";
    case Source::COURSE:
      return "course";
    case Source::NONE:
      break;
  }
  return "";
}

struct Epoch {
  double baseline_length = kNaN;
  double baseline_stat = kNaN;
  double heading_nis = kNaN;
  double course_residual = kNaN;
  double course_stat = kNaN;
  bool baseline_alarm = false;
  bool heading_alarm = false;
  bool course_alarm = false;
  bool trusted = true;            // false: do not fuse GNSS position or heading
  Source source = Source::NONE;   // test that took trust away
};

// The three tests together, plus the trust decision a navigation stack consumes.
//
// latch = true (flight use): this class only ever takes trust away. Once a test alarms, GNSS
// stays untrusted and the heading filter stays on the gyro until reset() is called. Healthy
// statistics are not a reason to trust again: a replayed baseline looks right every time the
// drone flies the heading the spoofer froze, and an attitude-aware array looks right always.
// The navigation filter, which can compare GNSS with its own coasted position, owns reset().
//
// latch = false (log analysis): every epoch is judged on its own, as in the Python benchmark.
class Monitor {
 public:
  explicit Monitor(double baseline = kBaseline, double sigma_slip = kSigmaSlip, bool latch = true)
      : baseline_(baseline), latch_(latch), course_(sigma_slip) {}

  bool initialised() const { return initialised_; }
  bool trusted() const { return trusted_; }
  Source source() const { return source_; }
  const HeadingFilter& heading_filter() const { return filter_; }

  void init(double heading0) {
    filter_ = HeadingFilter(heading0);
    initialised_ = true;
  }

  void reset() {
    trusted_ = true;
    source_ = Source::NONE;
    bad_baseline_ = 0;
    bad_heading_ = 0;
  }

  // Yaw-rate gyro sample (rad/s, positive from x towards y) covering dt seconds.
  void gyro(double rate, double dt) {
    if (initialised_) filter_.predict(rate, dt);
  }

  // One GNSS epoch. (bx, by) is the vector from the primary to the secondary antenna in the
  // local level frame and sigma_b its per-axis 1-sigma; (vx, vy) and c00, c01, c11 are the
  // navigation filter's horizontal velocity and its covariance; expected_drift is the commanded
  // angle between heading and course (heading minus course), zero in nose-first flight.
  Epoch epoch(bool has_baseline, double bx, double by, double sigma_b, double vx, double vy, double c00,
              double c01, double c11, double expected_drift = 0.0) {
    Epoch out;
    if (has_baseline && !initialised_) init(std::atan2(by, bx));
    if (!has_baseline) {
      const auto c = course_.step(vx, vy, c00, c01, c11, kNaN, kNaN, expected_drift);
      out.course_residual = c.residual;
      out.course_stat = c.stat;
      bad_baseline_ = 0;
      bad_heading_ = 0;
    } else {
      out.baseline_length = std::hypot(bx, by);
      const auto h = filter_.step(bx, by, sigma_b, baseline_, !latch_ || trusted_);
      out.baseline_stat = h.baseline_stat;
      out.heading_nis = h.heading_nis;
      // a heading taken from a baseline of the wrong length means nothing
      const double heading = h.baseline_stat < kThreshold ? std::atan2(by, bx) : kNaN;
      const auto c = course_.step(vx, vy, c00, c01, c11, heading, sigma_b / baseline_, expected_drift);
      out.course_residual = c.residual;
      out.course_stat = c.stat;
      bad_baseline_ = h.baseline_stat > kThreshold ? bad_baseline_ + 1 : 0;
      bad_heading_ = h.heading_nis > kThreshold ? bad_heading_ + 1 : 0;
    }
    out.baseline_alarm = bad_baseline_ >= kPersist;
    out.heading_alarm = bad_heading_ >= kPersist;
    out.course_alarm = std::isfinite(out.course_stat) && out.course_stat > kThreshold;
    const bool alarm = out.baseline_alarm || out.heading_alarm || out.course_alarm;
    const Source now = out.baseline_alarm ? Source::BASELINE
                       : out.heading_alarm ? Source::HEADING
                       : out.course_alarm ? Source::COURSE
                       : Source::NONE;
    if (!latch_) {
      trusted_ = !alarm;
      source_ = now;
    } else if (trusted_ && alarm) {
      trusted_ = false;
      source_ = now;
    }
    out.trusted = trusted_;
    out.source = source_;
    return out;
  }

 private:
  double baseline_;
  bool latch_;
  HeadingFilter filter_;
  CourseCheck course_;
  bool initialised_ = false;
  bool trusted_ = true;
  Source source_ = Source::NONE;
  int bad_baseline_ = 0;
  int bad_heading_ = 0;
};

}  // namespace gim
