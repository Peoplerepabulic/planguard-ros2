#include "planguard_control/brake_filter.hpp"

#include <algorithm>
#include <limits>

namespace planguard
{

double brake_speed_limit(double clearance, const BrakeParams & p)
{
  if (clearance <= p.d_stop) {
    return 0.0;
  }
  if (clearance >= p.d_slow) {
    return p.v_max;
  }
  return p.v_max * (clearance - p.d_stop) / (p.d_slow - p.d_stop);
}

BrakeResult brake_filter(const Cmd & ref, const std::vector<Vec2> & pts, const BrakeParams & p)
{
  BrakeResult out;
  out.cmd = ref;
  out.clearance = std::numeric_limits<double>::infinity();

  const double corridor = p.half_width + p.lateral_margin;
  for (const auto & q : pts) {
    if (q.x > p.front_x && std::abs(q.y) <= corridor) {
      out.clearance = std::min(out.clearance, q.x - p.front_x);
    }
  }

  if (ref.v > 0.0) {
    const double v_lim = brake_speed_limit(out.clearance, p);
    if (ref.v > v_lim) {
      out.cmd.v = v_lim;
      out.intervened = true;
    }
  }
  return out;
}

}  // namespace planguard
