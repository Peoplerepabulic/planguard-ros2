#pragma once

#include <vector>

#include "planguard_control/types.hpp"

namespace planguard
{

// Baseline shield: stop/slow-down zones in a forward corridor (same idea as the
// Stop/Slowdown polygons of nav2_collision_monitor). Distances are measured from the
// front bumper, not from the lidar. Angular velocity is never modified.
struct BrakeParams
{
  double front_x{0.20};          // bumper x in base frame
  double half_width{0.23};       // half of the collision footprint width (wheel spheres)
  double lateral_margin{0.05};
  double d_stop{0.20};           // clearance at/below which v := 0
  double d_slow{0.70};           // linear ramp between d_stop and d_slow
  double v_max{0.5};
};

struct BrakeResult
{
  Cmd cmd;
  bool intervened{false};
  double clearance{0.0};  // free distance ahead of the bumper inside the corridor (inf if none)
};

// Forward-speed cap as a function of clearance (exposed for testing).
double brake_speed_limit(double clearance, const BrakeParams & p);

BrakeResult brake_filter(const Cmd & ref, const std::vector<Vec2> & pts, const BrakeParams & p);

}  // namespace planguard
