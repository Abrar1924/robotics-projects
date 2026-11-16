// ROS 2 node around gim::Monitor (include/gim_monitor/dual_antenna.hpp).
//
// In (ENU, angles from east towards north):
//   /gim/imu             sensor_msgs/Imu                           angular_velocity.z = yaw rate (rad/s)
//   /gim/baseline        geometry_msgs/Vector3Stamped              primary -> secondary antenna (m)
//   /gim/velocity        geometry_msgs/TwistWithCovarianceStamped  navigation filter velocity (m/s)
//   /gim/expected_drift  std_msgs/Float64                          commanded heading minus course (rad)
// Out:
//   /gim/gnss_trusted    std_msgs/Bool      false: do not fuse GNSS position or heading
//   /gim/alarm_source    std_msgs/String    "", "baseline", "heading" or "course"
//   /diagnostics         diagnostic_msgs/DiagnosticArray   the statistics behind the decision
// Service:
//   /gim/reset           std_srvs/Trigger   trust GNSS again. Every alarm is held until this is
//                                           called: the node sees no position, so it cannot tell
//                                           a spoofer that went quiet from one that got better.
//
// A baseline and a velocity message belong to one GNSS epoch when their stamps are within
// 50 ms. Gyro samples are queued and integrated up to the epoch's stamp before it is tested,
// so the result does not depend on the order in which the three topics are delivered.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "gim_monitor/dual_antenna.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace {

constexpr double kSameEpoch = 0.05;  // s
constexpr double kMaxGyroGap = 0.5;  // s: a longer gap between gyro samples is not integrated

double seconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

diagnostic_msgs::msg::KeyValue pair(const std::string& key, double value) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.6g", value);
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = text;
  return kv;
}

}  // namespace

class MonitorNode : public rclcpp::Node {
 public:
  MonitorNode() : Node("gim_monitor") {
    const double length = declare_parameter("baseline_length", gim::kBaseline);
    sigma_b_ = declare_parameter("baseline_sigma", gim::kSigmaB);
    const double slip = declare_parameter("slip_sigma_deg", 1.0) * gim::kPi / 180.0;
    const bool latch = declare_parameter("latch", true);
    monitor_ = std::make_unique<gim::Monitor>(length, slip, latch);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/gim/imu", qos, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
          gyro_.push_back({seconds(msg->header.stamp), msg->angular_velocity.z});
        });
    baseline_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/gim/baseline", qos,
        [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { on_baseline(*msg); });
    velocity_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "/gim/velocity", qos,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) { on_velocity(*msg); });
    drift_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/gim/expected_drift", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { expected_drift_ = msg->data; });
    trusted_pub_ = create_publisher<std_msgs::msg::Bool>("/gim/gnss_trusted", qos);
    source_pub_ = create_publisher<std_msgs::msg::String>("/gim/alarm_source", qos);
    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", qos);
    reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "/gim/reset", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          monitor_->reset();
          response->success = true;
          response->message = "GNSS integrity alarm cleared";
        });
  }

 private:
  struct Gyro {
    double t;
    double rate;
  };
  struct Baseline {
    double t;
    double x;
    double y;
  };
  struct Velocity {
    builtin_interfaces::msg::Time stamp;
    double t;
    double vx;
    double vy;
    double c00;
    double c01;
    double c11;
  };

  void on_baseline(const geometry_msgs::msg::Vector3Stamped& msg) {
    baseline_ = Baseline{seconds(msg.header.stamp), msg.vector.x, msg.vector.y};
    if (pending_ && std::fabs(pending_->t - baseline_->t) < kSameEpoch) {
      process(*pending_, &*baseline_);
      pending_.reset();
    }
  }

  void on_velocity(const geometry_msgs::msg::TwistWithCovarianceStamped& msg) {
    const auto& c = msg.twist.covariance;
    const Velocity v{msg.header.stamp, seconds(msg.header.stamp), msg.twist.twist.linear.x,
                     msg.twist.twist.linear.y, c[0], c[1], c[7]};
    if (pending_) {  // the baseline for the previous epoch never came
      process(*pending_, nullptr);
      pending_.reset();
    }
    if (baseline_ && std::fabs(v.t - baseline_->t) < kSameEpoch) {
      process(v, &*baseline_);
    } else {
      pending_ = v;
    }
  }

  void integrate_gyro(double until) {
    while (!gyro_.empty() && gyro_.front().t <= until + 1e-9) {
      const Gyro g = gyro_.front();
      gyro_.pop_front();
      const double dt = g.t - last_gyro_t_;
      if (have_gyro_ && dt > 0.0 && dt <= kMaxGyroGap) monitor_->gyro(g.rate, dt);
      last_gyro_t_ = g.t;
      have_gyro_ = true;
    }
  }

  void process(const Velocity& v, const Baseline* b) {
    integrate_gyro(v.t);
    const gim::Epoch e = monitor_->epoch(b != nullptr, b ? b->x : 0.0, b ? b->y : 0.0, sigma_b_, v.vx, v.vy,
                                         v.c00, v.c01, v.c11, expected_drift_);
    std_msgs::msg::Bool trusted;
    trusted.data = e.trusted;
    trusted_pub_->publish(trusted);
    std_msgs::msg::String source;
    source.data = gim::source_name(e.source);
    source_pub_->publish(source);

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "gim_monitor: GNSS integrity (dual antenna)";
    status.hardware_id = "gnss";
    status.level = e.trusted ? diagnostic_msgs::msg::DiagnosticStatus::OK
                             : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = e.trusted ? "GNSS trusted" : "GNSS not trusted: " + source.data + " test";
    status.values.push_back(pair("baseline_length_m", e.baseline_length));
    status.values.push_back(pair("baseline_stat", e.baseline_stat));
    status.values.push_back(pair("heading_nis", e.heading_nis));
    status.values.push_back(pair("course_residual_deg", e.course_residual * 180.0 / gim::kPi));
    status.values.push_back(pair("course_stat", e.course_stat));
    status.values.push_back(pair("alarm_threshold", gim::kThreshold));
    diagnostic_msgs::msg::KeyValue source_kv;
    source_kv.key = "alarm_source";
    source_kv.value = source.data;
    status.values.push_back(source_kv);
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = v.stamp;
    array.status.push_back(status);
    diag_pub_->publish(array);
  }

  std::unique_ptr<gim::Monitor> monitor_;
  double sigma_b_ = gim::kSigmaB;
  double expected_drift_ = 0.0;
  std::deque<Gyro> gyro_;
  double last_gyro_t_ = 0.0;
  bool have_gyro_ = false;
  std::optional<Baseline> baseline_;
  std::optional<Velocity> pending_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr baseline_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drift_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trusted_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
