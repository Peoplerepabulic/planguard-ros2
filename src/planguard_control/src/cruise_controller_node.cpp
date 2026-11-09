// cruise_controller: the deliberately unreliable proposer. Drives straight at the goal
// from wheel odometry and never reads the lidar; obstacle handling is the shield's job.
#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "planguard_control/cruise_law.hpp"
#include "rclcpp/rclcpp.hpp"

namespace planguard
{

class CruiseController : public rclcpp::Node
{
public:
  CruiseController()
  : Node("cruise_controller")
  {
    goal_ = {declare_parameter("goal_x", 3.0), declare_parameter("goal_y", 3.0)};
    // Odometry starts at the spawn pose; these map odom -> world.
    origin_ = {declare_parameter("start_x", -3.0), declare_parameter("start_y", -3.0),
      declare_parameter("start_yaw", 0.0)};
    p_.k_linear = declare_parameter("k_linear", p_.k_linear);
    p_.k_angular = declare_parameter("k_angular", p_.k_angular);
    p_.v_max = declare_parameter("v_max", p_.v_max);
    p_.w_max = declare_parameter("w_max", p_.w_max);
    p_.goal_tolerance = declare_parameter("goal_tolerance", p_.goal_tolerance);
    p_.align_threshold = declare_parameter("align_threshold", p_.align_threshold);
    const double rate = declare_parameter("rate_hz", 20.0);

    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_raw", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {odom_ = m;});
    // Node clock -> follows /clock when use_sim_time is true.
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate), [this] {tick();});
    RCLCPP_INFO(get_logger(), "goal=(%.2f, %.2f) origin=(%.2f, %.2f, %.2f)",
      goal_.x, goal_.y, origin_.x, origin_.y, origin_.yaw);
  }

private:
  void tick()
  {
    if (!odom_) {
      return;
    }
    const auto & q = odom_->pose.pose.orientation;
    const double oyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double ox = odom_->pose.pose.position.x;
    const double oy = odom_->pose.pose.position.y;
    const double c = std::cos(origin_.yaw);
    const double s = std::sin(origin_.yaw);
    const Pose2 pose{origin_.x + c * ox - s * oy, origin_.y + s * ox + c * oy,
      wrap_angle(origin_.yaw + oyaw)};

    const auto out = cruise_law(pose, goal_, p_);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = out.cmd.v;
    cmd.angular.z = out.cmd.w;
    pub_->publish(cmd);

    if (out.reached && !reached_) {
      RCLCPP_INFO(get_logger(), "GOAL_REACHED t=%.2f pos=(%.3f, %.3f) dist=%.3f",
        now().seconds(), pose.x, pose.y, out.distance);
    }
    reached_ = out.reached;
  }

  Vec2 goal_;
  Pose2 origin_;
  CruiseParams p_;
  bool reached_{false};
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace planguard

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<planguard::CruiseController>());
  rclcpp::shutdown();
  return 0;
}
