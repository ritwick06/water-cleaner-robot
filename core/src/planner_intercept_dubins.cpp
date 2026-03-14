#include "awscr/planner_intercept_dubins.hpp"

#include <cmath>

namespace awscr {

namespace {

inline double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace

InterceptPlannerDubins::InterceptPlannerDubins(double turning_radius_m,
                                               double vessel_speed_mps)
    : R_(turning_radius_m), v_(vessel_speed_mps) {}

std::optional<InterceptPlan> InterceptPlannerDubins::plan(
    const StateEstimate& vessel_state,
    const EnuPoint& target_pos,
    const std::array<double, 2>& target_vel_enu,
    double t_horizon_s) const {
  // Simple time-discretized search: sample possible interception times
  // and choose the one where straight-line distance is reachable.
  const double dt = 1.0;
  const double psi0 = vessel_state.psi_rad;
  const double x0 = vessel_state.x_m;
  const double y0 = vessel_state.y_m;

  InterceptPlan best{};
  bool have_best = false;

  for (double t = dt; t <= t_horizon_s; t += dt) {
    const double xT =
        target_pos.x + target_vel_enu[0] * (t - 0.0);
    const double yT =
        target_pos.y + target_vel_enu[1] * (t - 0.0);
    const double dx = xT - x0;
    const double dy = yT - y0;
    const double dist = std::hypot(dx, dy);
    const double travel = v_ * t;
    if (travel < dist) continue;

    const double bearing = std::atan2(dy, dx);
    const double dpsi = std::fabs(wrapAngle(bearing - psi0));
    const double min_turn_length = R_ * dpsi;
    if (min_turn_length + (dist - R_) > travel + 1e-3) continue;

    InterceptPlan cand{};
    cand.intercept_time_s = t;
    cand.segments.push_back(
        {DubinsSegmentType::LEFT, min_turn_length});
    cand.segments.push_back(
        {DubinsSegmentType::STRAIGHT, dist - R_});

    if (!have_best || t < best.intercept_time_s) {
      best = cand;
      have_best = true;
    }
  }

  if (!have_best) return std::nullopt;
  return best;
}

}  // namespace awscr

