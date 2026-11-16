// ROS 2 node around mower::Supervisor.
//
// In:
//   /gnss/fix           sensor_msgs/NavSatFix   position covariance gives the 1-sigma error
//   /gnss/rtk_quality   std_msgs/UInt8          NMEA GGA quality indicator (4 fixed, 5 float).
//                                               Optional. Without it the node falls back to
//                                               NavSatStatus, which cannot tell fixed from float.
//   /hazard_clearance   std_msgs/Float64        distance to the nearest hazard (m)
//   /mower/command      std_msgs/String         "retreat": the mission layer sends the mower back
//                                               to its last RTK-fixed spot
// Out:
//   /mower/speed_limit  std_msgs/Float64        m/s; 0 means stop
//   /mower/mode         std_msgs/String         NOMINAL, DEGRADED, HOLD or RETREAT
//   /mower/event        std_msgs/String         "skip" when a hold has timed out
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "supervisor.hpp"

class MowerSupervisorNode : public rclcpp::Node {
 public:
  MowerSupervisorNode() : Node("mower_supervisor") {
    quality_timeout_ = declare_parameter("rtk_quality_timeout", 2.0);
    clearance_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/hazard_clearance", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { clearance_m_ = msg->data; });
    quality_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/gnss/rtk_quality", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          gga_quality_ = msg->data;
          quality_time_ = now();
          have_quality_ = true;
        });
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss/fix", 10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { on_fix(msg); });
    command_sub_ = create_subscription<std_msgs::msg::String>(
        "/mower/command", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == "retreat") {
            supervisor_.force(mower::Mode::RETREAT);
            publish_state();
          }
        });
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("/mower/speed_limit", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/mower/mode", 10);
    event_pub_ = create_publisher<std_msgs::msg::String>("/mower/event", 10);
  }

 private:
  void on_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    double t = static_cast<double>(msg->header.stamp.sec) +
               static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    if (t <= 0.0) t = now().seconds();  // a driver that leaves the stamp empty
    // The larger of the east and north variances: a fix that is good along a wall and poor
    // across it must count as poor.
    double variance = 0.0;
    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      variance = std::max(msg->position_covariance[0], msg->position_covariance[4]);
    }
    const double sigma = std::sqrt(std::max(variance, 0.0));
    const bool fresh = have_quality_ && (now() - quality_time_).seconds() <= quality_timeout_;
    const int quality = fresh ? mower::quality_from_gga(gga_quality_)
                              : mower::quality_from_status(msg->status.status);
    supervisor_.update(t, quality, sigma, clearance_m_);
    publish_state();
    if (supervisor_.hold_expired) {
      std_msgs::msg::String event;
      event.data = "skip";
      event_pub_->publish(event);
    }
  }

  void publish_state() {
    std_msgs::msg::Float64 speed;
    speed.data = supervisor_.speed_limit();
    speed_pub_->publish(speed);
    std_msgs::msg::String name;
    name.data = mower::mode_name(supervisor_.mode);
    mode_pub_->publish(name);
  }

  mower::Supervisor supervisor_;
  double clearance_m_ = 10.0;
  double quality_timeout_ = 2.0;
  std::uint8_t gga_quality_ = 0;
  bool have_quality_ = false;
  rclcpp::Time quality_time_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr clearance_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr quality_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MowerSupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
