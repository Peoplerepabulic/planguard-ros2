#pragma once

#include <optional>
#include <vector>

#include "planguard_control/types.hpp"

namespace planguard
{

// Half-plane a0*u0 + a1*u1 <= b.
struct HalfPlane
{
  double a0{0.0};
  double a1{0.0};
  double b{0.0};
};

// Exact solver for
//   min  w0 (u0 - r0)^2 + w1 (u1 - r1)^2   s.t.  a_i . u <= b_i,   w0, w1 > 0.
// In 2-D the unique minimiser is the unconstrained point, the projection onto one
// constraint line, or a vertex of two lines. All candidates are enumerated, sorted by
// cost, and the first feasible one is returned. Returns nullopt if the set is empty.
std::optional<Vec2> solve_qp2d(
  const Vec2 & ref, double w0, double w1, const std::vector<HalfPlane> & cons,
  double tol = 1e-9);

}  // namespace planguard
