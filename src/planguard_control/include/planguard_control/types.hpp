#pragma once

#include <cmath>
#include <vector>

namespace planguard
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

// Unicycle command: linear v [m/s], angular w [rad/s].
struct Cmd
{
  double v{0.0};
  double w{0.0};
};

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double wrap_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// LaserScan ranges -> points in the base frame. Lidar assumed level and yaw-aligned with base.
// Invalid returns (nan, inf, outside [range_min, range_max]) are dropped.
std::vector<Vec2> scan_to_points(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double lidar_x, double lidar_y);

}  // namespace planguard
