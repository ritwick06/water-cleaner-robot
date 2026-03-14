#pragma once

#include "controller_smc.hpp"
#include "dynamics_3dof.hpp"
#include "estimator_es_ekf.hpp"
#include "mission_fsm.hpp"
#include "planner_bcd.hpp"
#include "planner_intercept_dubins.hpp"
#include "vfh_plus.hpp"
#include "types.hpp"

namespace awscr {

// High-level façade around estimator, planners, controller, and FSM.
class Brain {
 public:
  explicit Brain(const VesselParams& vessel_params);

  void set_reference_origin(double lat0_deg, double lon0_deg, double alt0_m);

  // Coverage polygon (WGS84) from UI.
  void set_coverage_polygon(const std::vector<GpsFix>& poly_wgs84);

  // Sensor feeds from HAL.
  void feed_imu(const ImuSample& imu, double dt_s);
  void feed_gps(const GpsFix& gps);
  void feed_mag(const MagSample& mag);
  void handle_command(const Command& cmd);

  // Main control step at fixed rate.
  MotorThrustCmd step(double t_s);

  Telemetry get_telemetry() const;

 private:
  VesselParams vessel_params_;
  EsEkfEstimator estimator_;
  SmcController controller_;
  MissionFsm fsm_;

  BoustrophedonPlanner coverage_planner_;
  InterceptPlannerDubins intercept_planner_;
  VfhPlus vfh_;

  CoveragePlan active_coverage_;
  std::size_t current_lane_idx_{0};

  // Simple straight-line reference for now; later replaced by planners.
  TrajectoryPoint current_ref_{};

  // Origin for ENU to WGS84.
  double lat0_deg_{0.0};
  double lon0_deg_{0.0};
};

}  // namespace awscr

