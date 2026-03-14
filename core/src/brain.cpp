#include "awscr/brain.hpp"

#include <cmath>

namespace awscr {

namespace {

inline double rad2deg(double r) { return r * 180.0 / M_PI; }

// Very small ENU → WGS84 inverse suitable for lake-scale areas.
inline void enuToWgs84(double x_e, double y_n,
                       double lat0_deg, double lon0_deg,
                       double& lat_deg, double& lon_deg) {
  constexpr double Re = 6378137.0;
  const double lat0_rad = lat0_deg * M_PI / 180.0;
  (void)lon0_deg;
  const double d_lat = y_n / Re;
  const double d_lon = x_e / (Re * std::cos(lat0_rad));
  const double lat = lat0_rad + d_lat;
  const double lon = d_lon;
  lat_deg = lat * 180.0 / M_PI;
  lon_deg = lon * 180.0 / M_PI;
}

}  // namespace

Brain::Brain(const VesselParams& vessel_params)
    : vessel_params_(vessel_params),
      controller_(vessel_params),
      coverage_planner_(0.6),
      intercept_planner_(1.0, 1.0),
      vfh_(M_PI / 36.0, 2.0) {}

void Brain::set_reference_origin(double lat0_deg, double lon0_deg,
                                 double alt0_m) {
  lat0_deg_ = lat0_deg;
  lon0_deg_ = lon0_deg;
  estimator_.set_reference_origin(lat0_deg, lon0_deg, alt0_m);
  coverage_planner_.set_reference_origin(lat0_deg, lon0_deg, alt0_m);
}

void Brain::set_coverage_polygon(const std::vector<GpsFix>& poly_wgs84) {
  active_coverage_ = coverage_planner_.plan(poly_wgs84);
  current_lane_idx_ = 0;
  fsm_.set_has_plan(!active_coverage_.lanes.empty());
}

void Brain::feed_imu(const ImuSample& imu, double dt_s) {
  estimator_.predict(imu, dt_s);
}

void Brain::feed_gps(const GpsFix& gps) { estimator_.update_gps(gps); }

void Brain::feed_mag(const MagSample& mag) { estimator_.update_mag(mag); }

void Brain::handle_command(const Command& cmd) { fsm_.handle_command(cmd); }

MotorThrustCmd Brain::step(double t_s) {
  StateEstimate est = estimator_.get_state();
  current_ref_.t = t_s;

  if (fsm_.mode() == FsmMode::EXECUTING_COVERAGE &&
      current_lane_idx_ < active_coverage_.lanes.size()) {
    const auto& lane = active_coverage_.lanes[current_lane_idx_];
    const double dx = lane.end.x - lane.start.x;
    const double dy = lane.end.y - lane.start.y;
    const double lane_heading = std::atan2(dy, dx);
    const double px = est.x_m - lane.start.x;
    const double py = est.y_m - lane.start.y;
    const double proj =
        (px * dx + py * dy) / (dx * dx + dy * dy + 1e-6);
    if (proj > 1.05) {
      ++current_lane_idx_;
    }

    current_ref_.x = lane.end.x;
    current_ref_.y = lane.end.y;
    current_ref_.psi = lane_heading;
    current_ref_.u = 0.7;
    current_ref_.r = 0.0;
  } else {
    current_ref_.x = est.x_m + std::cos(est.psi_rad) * 1.0;
    current_ref_.y = est.y_m + std::sin(est.psi_rad) * 1.0;
    current_ref_.psi = est.psi_rad;
    current_ref_.u = 0.5;
    current_ref_.r = 0.0;
  }

  MotorThrustCmd cmd = controller_.compute(est, current_ref_, 0.02);
  cmd.t = t_s;
  return cmd;
}

Telemetry Brain::get_telemetry() const {
  Telemetry te{};
  const StateEstimate est = estimator_.get_state();
  double lat = 0.0, lon = 0.0;
  enuToWgs84(est.x_m, est.y_m, lat0_deg_, lon0_deg_, lat, lon);

  te.t = est.t;
  te.lat_deg = lat;
  te.lon_deg = lon;
  te.heading_deg = rad2deg(est.psi_rad);
  te.speed_mps = std::sqrt(est.u_mps * est.u_mps + est.v_mps * est.v_mps);
  te.mode = fsm_.mode();
  return te;
}

}  // namespace awscr

