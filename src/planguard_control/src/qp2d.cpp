#include "planguard_control/qp2d.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace planguard
{

namespace
{
struct Line  // normalised constraint in scaled coordinates: n . z <= c, |n| = 1
{
  double n0, n1, c;
};
}  // namespace

std::optional<Vec2> solve_qp2d(
  const Vec2 & ref, double w0, double w1, const std::vector<HalfPlane> & cons, double tol)
{
  // Scale z = W^{1/2} u so the objective becomes |z - z_ref|^2.
  const double s0 = std::sqrt(w0);
  const double s1 = std::sqrt(w1);
  const Vec2 zr{ref.x * s0, ref.y * s1};

  std::vector<Line> lines;
  lines.reserve(cons.size());
  for (const auto & h : cons) {
    const double n0 = h.a0 / s0;
    const double n1 = h.a1 / s1;
    const double norm = std::hypot(n0, n1);
    if (norm < 1e-12) {
      if (h.b < -tol) {
        return std::nullopt;  // 0 <= b violated
      }
      continue;
    }
    lines.push_back({n0 / norm, n1 / norm, h.b / norm});
  }

  auto feasible = [&](const Vec2 & z) {
      for (const auto & l : lines) {
        if (l.n0 * z.x + l.n1 * z.y > l.c + tol) {
          return false;
        }
      }
      return true;
    };

  std::vector<std::pair<double, Vec2>> cand;
  cand.reserve(1 + lines.size() + lines.size() * lines.size() / 2);
  cand.push_back({0.0, zr});
  for (const auto & l : lines) {
    const double viol = l.n0 * zr.x + l.n1 * zr.y - l.c;
    const Vec2 z{zr.x - viol * l.n0, zr.y - viol * l.n1};
    cand.push_back({viol * viol, z});
  }
  for (std::size_t i = 0; i < lines.size(); ++i) {
    for (std::size_t j = i + 1; j < lines.size(); ++j) {
      const auto & a = lines[i];
      const auto & b = lines[j];
      const double det = a.n0 * b.n1 - a.n1 * b.n0;
      if (std::abs(det) < 1e-12) {
        continue;
      }
      const Vec2 z{(a.c * b.n1 - a.n1 * b.c) / det, (a.n0 * b.c - a.c * b.n0) / det};
      const double dx = z.x - zr.x;
      const double dy = z.y - zr.y;
      cand.push_back({dx * dx + dy * dy, z});
    }
  }

  std::sort(cand.begin(), cand.end(), [](const auto & p, const auto & q) {return p.first < q.first;});
  for (const auto & c : cand) {
    if (feasible(c.second)) {
      return Vec2{c.second.x / s0, c.second.y / s1};
    }
  }
  return std::nullopt;
}

}  // namespace planguard
