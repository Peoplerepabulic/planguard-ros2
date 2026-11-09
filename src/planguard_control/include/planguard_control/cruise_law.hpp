#pragma once

#include "planguard_control/types.hpp"

namespace planguard
{

// Go-to-goal law for the "unreliable proposer". It never looks at the lidar.
struct CruiseParams
{
  double k_linear{0.5};
  double k_angular{1.5};
  double v_max{0.5};
  double w_max{1.5};
  double goal_tolerance{0.3};
  double align_threshold{0.8};  // |heading error| above this -> rotate in place
};

struct CruiseOutput
{
  Cmd cmd;
  bool reached{false};
  double distance{0.0};
  double heading_error{0.0};
};

CruiseOutput cruise_law(const Pose2 & pose, const Vec2 & goal, const CruiseParams & p);

}  // namespace planguard
