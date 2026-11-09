#include "planguard_control/cbf_filter.hpp"

#include <algorithm>
#include <limits>
#include <map>

namespace planguard
{

namespace
{
double circle_radius(const CbfParams & p, std::size_t k)
{
  return p.circle_r.size() == 1 ? p.circle_r[0] : p.circle_r.at(k);
}

// Point at distance R from circle k is inactive for every admissible u when
//   (alpha/2)(R^2 - d_k^2) >= R * (max|v| + |c_k| * w_max).
double relevance_radius(const CbfParams & p, std::size_t k)
{
  const double d = circle_radius(p, k) + p.margin;
  const double s = std::max(std::abs(p.v_min), std::abs(p.v_max)) + std::abs(p.circle_x[k]) * p.w_max;
  return (s + std::sqrt(s * s + p.alpha * p.alpha * d * d)) / p.alpha;
}
}  // namespace

double cbf_relevance_radius(const CbfParams & p)
{
  double r = 0.0;
  for (std::size_t k = 0; k < p.circle_x.size(); ++k) {
    r = std::max(r, relevance_radius(p, k));
  }
  return r;
}

std::vector<HalfPlane> cbf_constraints(
  const std::vector<Vec2> & pts, const CbfParams & p, double * min_h)
{
  const double bin = p.bin_deg * M_PI / 180.0;
  double h_min = std::numeric_limits<double>::infinity();

  std::vector<HalfPlane> cons;
  for (std::size_t k = 0; k < p.circle_x.size(); ++k) {
    const double c = p.circle_x[k];
    const double d = circle_radius(p, k) + p.margin;
    const double R = relevance_radius(p, k);
    std::map<int, Vec2> nearest;  // angular bin -> r with the smallest norm
    for (const auto & o : pts) {
      const Vec2 r{o.x - c, o.y};
      const double n = std::hypot(r.x, r.y);
      if (n > R) {
        continue;
      }
      const int key = static_cast<int>(std::floor(std::atan2(r.y, r.x) / bin));
      auto it = nearest.find(key);
      if (it == nearest.end() || n < std::hypot(it->second.x, it->second.y)) {
        nearest[key] = r;
      }
    }
    for (const auto & kv : nearest) {
      const Vec2 & r = kv.second;
      const double h = r.x * r.x + r.y * r.y - d * d;
      h_min = std::min(h_min, h);
      cons.push_back({r.x, c * r.y, 0.5 * p.alpha * h});
    }
  }
  if (min_h) {
    *min_h = h_min;
  }
  return cons;
}

CbfResult cbf_filter(const Cmd & ref, const std::vector<Vec2> & pts, const CbfParams & p)
{
  CbfResult out;
  auto cons = cbf_constraints(pts, p, &out.min_h);
  out.n_constraints = cons.size();
  cons.push_back({1.0, 0.0, p.v_max});
  cons.push_back({-1.0, 0.0, -p.v_min});
  cons.push_back({0.0, 1.0, p.w_max});
  cons.push_back({0.0, -1.0, p.w_max});

  const auto u = solve_qp2d({ref.v, ref.w}, 1.0, p.w_weight, cons);
  if (!u) {
    out.fallback = true;
    out.intervened = true;
    out.cmd = {0.0, 0.0};
    return out;
  }
  out.cmd = {u->x, u->y};
  out.intervened = std::abs(out.cmd.v - ref.v) > 1e-3 || std::abs(out.cmd.w - ref.w) > 1e-3;
  return out;
}

}  // namespace planguard
