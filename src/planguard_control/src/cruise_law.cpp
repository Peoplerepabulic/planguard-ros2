#include "planguard_control/cruise_law.hpp"

#include <algorithm>

namespace planguard
{

CruiseOutput cruise_law(const Pose2 & pose, const Vec2 & goal, const CruiseParams & p)
{
  CruiseOutput out;
  const double dx = goal.x - pose.x;
  const double dy = goal.y - pose.y;
  out.distance = std::hypot(dx, dy);
  out.heading_error = wrap_angle(std::atan2(dy, dx) - pose.yaw);

  if (out.distance < p.goal_tolerance) {
    out.reached = true;
    return out;  // zero command
  }

  out.cmd.w = std::clamp(p.k_angular * out.heading_error, -p.w_max, p.w_max);
  if (std::abs(out.heading_error) > p.align_threshold) {
    out.cmd.v = 0.0;  // align first
  } else {
    // slow down near the goal and when not facing it
    out.cmd.v = std::min(p.v_max, p.k_linear * out.distance) * std::cos(out.heading_error);
  }
  return out;
}

}  // namespace planguard
