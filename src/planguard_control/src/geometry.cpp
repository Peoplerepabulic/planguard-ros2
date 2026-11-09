#include "planguard_control/types.hpp"

namespace planguard
{

std::vector<Vec2> scan_to_points(
  const std::vector<float> & ranges, double angle_min, double angle_increment,
  double range_min, double range_max, double lidar_x, double lidar_y)
{
  std::vector<Vec2> pts;
  pts.reserve(ranges.size());
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const double r = ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max) {
      continue;
    }
    const double a = angle_min + static_cast<double>(i) * angle_increment;
    pts.push_back({lidar_x + r * std::cos(a), lidar_y + r * std::sin(a)});
  }
  return pts;
}

}  // namespace planguard
