#pragma once

#include "types.hpp"

#include <vector>

namespace awscr {

struct EnuPoint {
  double x;
  double y;
};

struct CoverageLane {
  EnuPoint start;
  EnuPoint end;
};

struct CoveragePlan {
  double sweep_angle_rad{0.0};
  std::vector<CoverageLane> lanes;
};

// Boustrophedon coverage planner operating in a local ENU frame.
// For now, we provide a minimal implementation:
//  - Convert polygon WGS84 → ENU.
//  - Sample candidate sweep angles.
//  - For best angle, fill polygon with parallel line segments (lanes).
class BoustrophedonPlanner {
 public:
  BoustrophedonPlanner(double lane_spacing_m, int angle_samples = 18);

  void set_reference_origin(double lat0_deg, double lon0_deg, double alt0_m);

  CoveragePlan plan(const std::vector<GpsFix>& polygon_vertices);

 private:
  double lane_spacing_m_;
  int angle_samples_;
  double lat0_rad_{0.0};
  double lon0_rad_{0.0};

  static void wgs84ToEnu(double lat_rad, double lon_rad,
                         double lat0_rad, double lon0_rad,
                         double& x_e, double& y_n);
  static bool pointInPolygon(const EnuPoint& p,
                             const std::vector<EnuPoint>& poly);
};

}  // namespace awscr

