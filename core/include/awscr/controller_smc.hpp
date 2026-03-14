#pragma once

#include "dynamics_3dof.hpp"
#include "types.hpp"

namespace awscr {

// Reference point for controller.
struct TrajectoryPoint {
  double t;
  double x;
  double y;
  double psi;
  double u;
  double r;
};

// Simple sliding-mode-like controller for surge and yaw.
class SmcController {
 public:
  explicit SmcController(const VesselParams& p);

  MotorThrustCmd compute(const StateEstimate& est,
                         const TrajectoryPoint& ref,
                         double dt_s);

 private:
  VesselParams p_;
  double integral_ex_{0.0};
  double integral_epsi_{0.0};
};

}  // namespace awscr

