#pragma once

#include "types.hpp"
#include "planner_bcd.hpp"

#include <optional>
#include <vector>

namespace awscr {

enum class DubinsSegmentType { LEFT, RIGHT, STRAIGHT };

struct DubinsSegment {
  DubinsSegmentType type;
  double length_m;
};

struct InterceptPlan {
  double intercept_time_s;
  std::vector<DubinsSegment> segments;
};

// Minimal Dubins-based dynamic interception planner.
class InterceptPlannerDubins {
 public:
  InterceptPlannerDubins(double turning_radius_m, double vessel_speed_mps);

  std::optional<InterceptPlan> plan(const StateEstimate& vessel_state,
                                    const EnuPoint& target_pos,
                                    const std::array<double, 2>& target_vel_enu,
                                    double t_horizon_s) const;

 private:
  double R_;
  double v_;
};

}  // namespace awscr

