// safety_shield: /cmd_vel_raw + /scan -> /cmd_vel.
// mode = off   : pass-through (kept in the loop so all conditions see identical latency)
// mode = brake : stop/slow-down corridor (baseline)
// mode = cbf   : CBF-QP filter on (v, w)
// Fail-safe: in brake/cbf mode, missing or stale scan -> zero command.
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "planguard_control/brake_filter.hpp"
#include "planguard_control/cbf_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int32.hpp"

namespace planguard
{

class SafetyShield : public rclcpp::Node
{
public:
  SafetyShield()
  : Node("safety_shield")
  {
    mode_ = declare_parameter("mode", std::string("cbf"));
    if (mode_ != "off" && mode_ != "brake" && mode_ != "cbf") {
      throw std::invalid_argument("mode must be off | brake | cbf, got " + mode_);
    }
    lidar_x_ = declare_parameter("lidar_x", 0.0);
    lidar_y_ = declare_parameter("lidar_y", 0.0);
    scan_timeout_ = declare_parameter("scan_timeout", 0.5);

    bp_.front_x = declare_parameter("brake.front_x", bp_.front_x);
    bp_.half_width = declare_parameter("brake.half_width", bp_.half_width);
    bp_.lateral_margin = declare_parameter("brake.lateral_margin", bp_.lateral_margin);
    bp_.d_stop = declare_parameter("brake.d_stop", bp_.d_stop);
    bp_.d_slow = declare_parameter("brake.d_slow", bp_.d_slow);
    bp_.v_max = declare_parameter("brake.v_max", bp_.v_max);

    cp_.circle_x = declare_parameter("cbf.circle_x", cp_.circle_x);
    cp_.circle_r = declare_parameter("cbf.circle_r", cp_.circle_r);
    if (cp_.circle_r.size() != 1 && cp_.circle_r.size() != cp_.circle_x.size()) {
      throw std::invalid_argument("cbf.circle_r must have 1 entry or one per cbf.circle_x");
    }
    cp_.margin = declare_parameter("cbf.margin", cp_.margin);
    cp_.alpha = declare_parameter("cbf.alpha", cp_.alpha);
    cp_.w_weight = declare_parameter("cbf.w_weight", cp_.w_weight);
    cp_.v_min = declare_parameter("cbf.v_min", cp_.v_min);
    cp_.v_max = declare_parameter("cbf.v_max", cp_.v_max);
    cp_.w_max = declare_parameter("cbf.w_max", cp_.w_max);
    cp_.bin_deg = declare_parameter("cbf.bin_deg", cp_.bin_deg);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    count_pub_ = create_publisher<std_msgs::msg::UInt32>("shield/interventions", 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("shield/active", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr m) {scan_ = m;});
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_raw", 10, [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_cmd(*m);});

    RCLCPP_INFO(get_logger(), "mode=%s  brake[d_stop=%.2f d_slow=%.2f]  cbf[alpha=%.2f circles=%zu margin=%.2f]",
      mode_.c_str(), bp_.d_stop, bp_.d_slow, cp_.alpha, cp_.circle_x.size(), cp_.margin);
  }

private:
  void on_cmd(const geometry_msgs::msg::Twist & raw)
  {
    const Cmd ref{raw.linear.x, raw.angular.z};
    Cmd out = ref;
    bool active = false;
    std::string why;

    if (mode_ != "off") {
      const bool stale = !scan_ ||
        (now() - rclcpp::Time(scan_->header.stamp, now().get_clock_type())).seconds() > scan_timeout_;
      if (stale) {
        out = {0.0, 0.0};
        active = true;
        why = "no_scan";
      } else {
        const auto pts = scan_to_points(
          scan_->ranges, scan_->angle_min, scan_->angle_increment,
          scan_->range_min, scan_->range_max, lidar_x_, lidar_y_);
        if (mode_ == "brake") {
          const auto r = brake_filter(ref, pts, bp_);
          out = r.cmd;
          active = r.intervened;
          why = "clearance=" + std::to_string(r.clearance);
        } else {
          const auto r = cbf_filter(ref, pts, cp_);
          out = r.cmd;
          active = r.intervened;
          why = (r.fallback ? "fallback min_h=" : "min_h=") + std::to_string(r.min_h);
        }
      }
    }

    if (active && !was_active_) {
      ++count_;
      RCLCPP_INFO(get_logger(),
        "SHIELD_INTERVENTION n=%u mode=%s t=%.2f v_ref=%.3f w_ref=%.3f -> v=%.3f w=%.3f %s",
        count_, mode_.c_str(), now().seconds(), ref.v, ref.w, out.v, out.w, why.c_str());
    }
    was_active_ = active;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = out.v;
    cmd.angular.z = out.w;
    pub_->publish(cmd);
    std_msgs::msg::UInt32 n;
    n.data = count_;
    count_pub_->publish(n);
    std_msgs::msg::Bool b;
    b.data = active;
    active_pub_->publish(b);
  }

  std::string mode_;
  double lidar_x_{0.0};
  double lidar_y_{0.0};
  double scan_timeout_{0.5};
  BrakeParams bp_;
  CbfParams cp_;
  uint32_t count_{0};
  bool was_active_{false};
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

}  // namespace planguard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planguard::SafetyShield>());
  rclcpp::shutdown();
  return 0;
}
