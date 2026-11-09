#pragma once

#include <cstddef>
#include <vector>

#include "planguard_control/qp2d.hpp"
#include "planguard_control/types.hpp"

namespace planguard
{

// CBF-QP safety filter for a unicycle. The footprint is covered by circles centred on
// the x-axis of the base frame at x = c_k with radius rho_k.
//
// For circle k and lidar point o (base frame), r = o - (c_k, 0) and
//   h = |r|^2 - d_k^2,          d_k = rho_k + margin
//   dh/dt = -2 (r_x v + c_k r_y w)   (static obstacle, body-frame kinematics)
// The CBF condition dh/dt >= -alpha h gives one linear constraint on u = (v, w):
//   r_x v + c_k r_y w <= (alpha/2) (|r|^2 - d_k^2)
// The filter solves  min (v - v_ref)^2 + w_weight (w - w_ref)^2  subject to all of
// them plus the actuator box. u = 0 satisfies every constraint whenever h > 0 for all
// points, so the QP is feasible as long as the robot is outside every safety disk.
struct CbfParams
{
  // Two circles at x = +-0.12 with r = 0.26 cover the collision footprint (chassis
  // 0.40 x 0.30 plus wheel spheres r 0.06 at y = +-0.17; checked numerically).
  // No circle at c = 0: its constraint r_x v <= b has no w term, so it can only brake.
  std::vector<double> circle_x{0.12, -0.12};
  std::vector<double> circle_r{0.26, 0.26};
  double margin{0.05};
  double alpha{1.5};
  // w_weight = c^2 makes the objective isotropic in the velocity of the look-ahead
  // point (v, c w): the QP is then the Euclidean projection of that velocity.
  double w_weight{0.0144};
  double v_min{-0.2};
  double v_max{0.5};
  double w_max{1.5};
  double bin_deg{5.0};   // keep the nearest point per angular bin per circle
};

struct CbfResult
{
  Cmd cmd;
  bool intervened{false};
  bool fallback{false};          // QP infeasible -> full stop
  double min_h{0.0};             // smallest barrier value among considered points
  std::size_t n_constraints{0};
};

// Smallest |r| beyond which a point can never be active for any admissible u
// (used to prune points without changing the solution).
double cbf_relevance_radius(const CbfParams & p);

std::vector<HalfPlane> cbf_constraints(
  const std::vector<Vec2> & pts, const CbfParams & p, double * min_h = nullptr);

CbfResult cbf_filter(const Cmd & ref, const std::vector<Vec2> & pts, const CbfParams & p);

}  // namespace planguard
