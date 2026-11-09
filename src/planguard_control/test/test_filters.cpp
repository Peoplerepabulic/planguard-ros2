#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "planguard_control/brake_filter.hpp"
#include "planguard_control/cbf_filter.hpp"
#include "planguard_control/cruise_law.hpp"
#include "planguard_control/qp2d.hpp"

using namespace planguard;  // NOLINT

namespace
{
// A straight wall of points perpendicular to x at distance x0, spanning |y| <= half.
std::vector<Vec2> wall_ahead(double x0, double half = 1.0, double step = 0.02)
{
  std::vector<Vec2> pts;
  for (double y = -half; y <= half + 1e-9; y += step) {
    pts.push_back({x0, y});
  }
  return pts;
}

// Small cluster of points (a thin pole) centred at (x, y).
std::vector<Vec2> pole(double x, double y, double r = 0.05)
{
  std::vector<Vec2> pts;
  for (int k = 0; k < 24; ++k) {
    const double a = 2.0 * M_PI * k / 24.0;
    pts.push_back({x + r * std::cos(a), y + r * std::sin(a)});
  }
  return pts;
}

bool satisfies(const Cmd & u, const std::vector<HalfPlane> & cons, double tol = 1e-7)
{
  for (const auto & h : cons) {
    if (h.a0 * u.v + h.a1 * u.w > h.b + tol) {
      return false;
    }
  }
  return true;
}
}  // namespace

// ---------------- scan conversion ----------------
TEST(Scan, DropsInvalidAndConvertsPolar)
{
  const float inf = std::numeric_limits<float>::infinity();
  const std::vector<float> r{1.0f, inf, 0.05f, 2.0f};
  const auto pts = scan_to_points(r, 0.0, M_PI / 2, 0.12, 8.0, 0.1, 0.0);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_NEAR(pts[0].x, 1.1, 1e-9);
  EXPECT_NEAR(pts[0].y, 0.0, 1e-9);
  EXPECT_NEAR(pts[1].x, 0.1, 1e-9);  // index 3 -> angle 3*pi/2 -> straight right
  EXPECT_NEAR(pts[1].y, -2.0, 1e-9);
}

// ---------------- cruise law ----------------
TEST(Cruise, RotatesInPlaceWhenMisaligned)
{
  const auto o = cruise_law({0, 0, 0}, {0, 3}, CruiseParams{});
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_GT(o.cmd.w, 0.0);
}

TEST(Cruise, DrivesForwardWhenAligned)
{
  const auto o = cruise_law({0, 0, 0}, {5, 0}, CruiseParams{});
  EXPECT_NEAR(o.cmd.v, 0.5, 1e-9);
  EXPECT_NEAR(o.cmd.w, 0.0, 1e-9);
}

TEST(Cruise, SlowsNearGoalAndStopsInsideTolerance)
{
  CruiseParams p;
  EXPECT_NEAR(cruise_law({0, 0, 0}, {0.6, 0}, p).cmd.v, 0.3, 1e-9);
  const auto o = cruise_law({0, 0, 0}, {0.2, 0}, p);
  EXPECT_TRUE(o.reached);
  EXPECT_DOUBLE_EQ(o.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(o.cmd.w, 0.0);
}

// ---------------- brake shield ----------------
TEST(Brake, AllClearPassesThrough)
{
  const auto r = brake_filter({0.5, 0.3}, wall_ahead(3.0), BrakeParams{});
  EXPECT_FALSE(r.intervened);
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.5);
  EXPECT_DOUBLE_EQ(r.cmd.w, 0.3);
}

TEST(Brake, SlowZoneIsLinear)
{
  BrakeParams p;
  const double clearance = 0.45;  // halfway between d_stop=0.20 and d_slow=0.70
  const auto r = brake_filter({0.5, 0.0}, wall_ahead(p.front_x + clearance), p);
  EXPECT_TRUE(r.intervened);
  EXPECT_NEAR(r.clearance, clearance, 1e-9);
  EXPECT_NEAR(r.cmd.v, 0.25, 1e-9);
}

TEST(Brake, StopZoneZeroesLinearButKeepsAngular)
{
  BrakeParams p;
  const auto r = brake_filter({0.5, 0.7}, wall_ahead(p.front_x + 0.1), p);
  EXPECT_TRUE(r.intervened);
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.0);
  EXPECT_DOUBLE_EQ(r.cmd.w, 0.7);
}

TEST(Brake, IgnoresPointsOutsideCorridor)
{
  BrakeParams p;
  // pole close ahead but laterally outside half_width + margin
  const auto r = brake_filter({0.5, 0.0}, pole(0.4, 0.40, 0.02), p);
  EXPECT_FALSE(r.intervened);
}

TEST(Brake, CornerPointThatA30DegSectorWouldMiss)
{
  BrakeParams p;
  // 4 cm ahead of the bumper, 0.14 m lateral: atan(0.14/0.24) = 30.3 deg from the lidar
  const auto r = brake_filter({0.5, 0.0}, {{0.24, 0.14}}, p);
  EXPECT_TRUE(r.intervened);
  EXPECT_DOUBLE_EQ(r.cmd.v, 0.0);
}

TEST(Brake, NeverBlocksReversing)
{
  const auto r = brake_filter({-0.1, 0.0}, wall_ahead(0.25), BrakeParams{});
  EXPECT_FALSE(r.intervened);
  EXPECT_DOUBLE_EQ(r.cmd.v, -0.1);
}

// ---------------- 2-D QP ----------------
TEST(QP2D, UnconstrainedOptimumIsReference)
{
  const auto u = solve_qp2d({0.3, -0.2}, 1.0, 0.1, {{1, 0, 1.0}});
  ASSERT_TRUE(u);
  EXPECT_NEAR(u->x, 0.3, 1e-12);
  EXPECT_NEAR(u->y, -0.2, 1e-12);
}

TEST(QP2D, SingleActiveConstraintWeightedProjection)
{
  // min (u0-1)^2 + 4 (u1-1)^2  s.t.  u0 + u1 <= 0
  // KKT: 2(u0-1) = -l, 8(u1-1) = -l, u0+u1 = 0 -> u0 = -0.6, u1 = 0.6
  const auto u = solve_qp2d({1.0, 1.0}, 1.0, 4.0, {{1, 1, 0.0}});
  ASSERT_TRUE(u);
  EXPECT_NEAR(u->x, -0.6, 1e-9);
  EXPECT_NEAR(u->y, 0.6, 1e-9);
}

TEST(QP2D, VertexSolution)
{
  const auto u = solve_qp2d({1.0, 1.0}, 1.0, 1.0, {{1, 0, 0.0}, {0, 1, 0.0}});
  ASSERT_TRUE(u);
  EXPECT_NEAR(u->x, 0.0, 1e-12);
  EXPECT_NEAR(u->y, 0.0, 1e-12);
}

TEST(QP2D, InfeasibleReturnsNullopt)
{
  EXPECT_FALSE(solve_qp2d({0, 0}, 1, 1, {{1, 0, -1.0}, {-1, 0, -1.0}}));
}

TEST(QP2D, MatchesBruteForceOnRandomProblems)
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> U(-1.0, 1.0);
  for (int trial = 0; trial < 200; ++trial) {
    std::vector<HalfPlane> cons;
    for (int k = 0; k < 6; ++k) {
      cons.push_back({U(rng), U(rng), 0.2 + 0.5 * std::abs(U(rng))});  // origin always feasible
    }
    const Vec2 ref{2.0 * U(rng), 2.0 * U(rng)};
    const double w1 = 0.1 + std::abs(U(rng));
    const auto u = solve_qp2d(ref, 1.0, w1, cons);
    ASSERT_TRUE(u);
    const double best = (u->x - ref.x) * (u->x - ref.x) + w1 * (u->y - ref.y) * (u->y - ref.y);
    for (const auto & h : cons) {
      ASSERT_LE(h.a0 * u->x + h.a1 * u->y, h.b + 1e-7);
    }
    // grid search over the feasible set never beats the exact answer
    for (double a = -3; a <= 3; a += 0.02) {
      for (double b = -3; b <= 3; b += 0.02) {
        bool ok = true;
        for (const auto & h : cons) {
          ok = ok && (h.a0 * a + h.a1 * b <= h.b);
        }
        if (ok) {
          const double c = (a - ref.x) * (a - ref.x) + w1 * (b - ref.y) * (b - ref.y);
          ASSERT_GE(c, best - 1e-9);
        }
      }
    }
  }
}

// ---------------- CBF shield ----------------
TEST(Cbf, NoObstaclesPassesThrough)
{
  const auto r = cbf_filter({0.5, 0.2}, {}, CbfParams{});
  EXPECT_FALSE(r.intervened);
  EXPECT_NEAR(r.cmd.v, 0.5, 1e-9);
  EXPECT_NEAR(r.cmd.w, 0.2, 1e-9);
}

TEST(Cbf, FarObstaclesArePrunedLosslessly)
{
  CbfParams p;
  const double R = cbf_relevance_radius(p);
  const auto r = cbf_filter({0.5, 0.0}, wall_ahead(R + 0.4), p);
  EXPECT_EQ(r.n_constraints, 0u);
  EXPECT_FALSE(r.intervened);
}

TEST(Cbf, WallAheadReducesSpeedAndSatisfiesEveryConstraint)
{
  CbfParams p;
  const auto pts = wall_ahead(0.55);
  const auto r = cbf_filter({0.5, 0.0}, pts, p);
  EXPECT_TRUE(r.intervened);
  EXPECT_LT(r.cmd.v, 0.5);
  EXPECT_TRUE(satisfies(r.cmd, cbf_constraints(pts, p)));
}

TEST(Cbf, OffsetObstacleIsSteeredAroundNotOnlyBraked)
{
  CbfParams p;
  // pole ahead-left: the filter should turn right (w < 0) rather than only slow down
  const auto r = cbf_filter({0.5, 0.0}, pole(0.6, 0.20), p);
  EXPECT_TRUE(r.intervened);
  EXPECT_LT(r.cmd.w, -1e-3);
  EXPECT_GT(r.cmd.v, 0.0);
}

TEST(Cbf, ZeroCommandAlwaysFeasibleOutsideSafetySet)
{
  CbfParams p;
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> U(-1.5, 1.5);
  for (int t = 0; t < 100; ++t) {
    std::vector<Vec2> pts;
    for (int k = 0; k < 40; ++k) {
      Vec2 q{U(rng), U(rng)};
      bool outside = true;
      for (std::size_t k = 0; k < p.circle_x.size(); ++k) {
        outside = outside && std::hypot(q.x - p.circle_x[k], q.y) > p.circle_r[k] + p.margin + 1e-3;
      }
      if (outside) {
        pts.push_back(q);
      }
    }
    EXPECT_TRUE(satisfies({0.0, 0.0}, cbf_constraints(pts, p)));
    const auto r = cbf_filter({U(rng), U(rng)}, pts, p);
    EXPECT_FALSE(r.fallback);
  }
}

TEST(Cbf, SymmetricHeadOnObstacleGivesNoTurn)
{
  // Known CBF-QP limitation: with a symmetric obstacle dead ahead and w_ref = 0 the
  // filter only brakes (no reason to prefer left or right) -> potential deadlock.
  CbfParams p;
  const auto r = cbf_filter({0.5, 0.0}, pole(0.5, 0.0), p);
  EXPECT_TRUE(r.intervened);
  EXPECT_NEAR(r.cmd.w, 0.0, 1e-6);
  EXPECT_LT(r.cmd.v, 0.5);
}

TEST(Cbf, RearCircleConstrainsTurningIntoSideObstacle)
{
  CbfParams p;
  // obstacle beside-behind on the right; turning left swings the tail right into it
  const std::vector<Vec2> pts = pole(-0.25, -0.33, 0.02);
  const auto cons = cbf_constraints(pts, p);
  EXPECT_FALSE(satisfies({0.0, 1.5}, cons));
  EXPECT_TRUE(satisfies({0.0, -1.5}, cons));
}
