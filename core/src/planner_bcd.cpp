#include "awscr/planner_bcd.hpp"

#include <cmath>

namespace awscr {

namespace {

constexpr double kReEarth = 6378137.0;

}  // namespace

BoustrophedonPlanner::BoustrophedonPlanner(double lane_spacing_m,
                                           int angle_samples)
    : lane_spacing_m_(lane_spacing_m), angle_samples_(angle_samples) {}

void BoustrophedonPlanner::set_reference_origin(double lat0_deg,
                                                double lon0_deg,
                                                double alt0_m) {
  (void)alt0_m;
  lat0_rad_ = lat0_deg * M_PI / 180.0;
  lon0_rad_ = lon0_deg * M_PI / 180.0;
}

void BoustrophedonPlanner::wgs84ToEnu(double lat_rad, double lon_rad,
                                      double lat0_rad, double lon0_rad,
                                      double& x_e, double& y_n) {
  const double d_lat = lat_rad - lat0_rad;
  const double d_lon = lon_rad - lon0_rad;
  const double c_lat0 = std::cos(lat0_rad);
  x_e = d_lon * c_lat0 * kReEarth;
  y_n = d_lat * kReEarth;
}

bool BoustrophedonPlanner::pointInPolygon(
    const EnuPoint& p, const std::vector<EnuPoint>& poly) {
  bool inside = false;
  const std::size_t n = poly.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const EnuPoint& pi = poly[i];
    const EnuPoint& pj = poly[j];
    const bool intersect =
        ((pi.y > p.y) != (pj.y > p.y)) &&
        (p.x < (pj.x - pi.x) * (p.y - pi.y) / (pj.y - pi.y + 1e-9) + pi.x);
    if (intersect) inside = !inside;
  }
  return inside;
}

CoveragePlan BoustrophedonPlanner::plan(
    const std::vector<GpsFix>& polygon_vertices) {
  CoveragePlan plan{};
  if (polygon_vertices.size() < 3) return plan;

  // Convert polygon to ENU.
  std::vector<EnuPoint> poly;
  poly.reserve(polygon_vertices.size());
  for (const auto& v : polygon_vertices) {
    const double lat = v.lat_deg * M_PI / 180.0;
    const double lon = v.lon_deg * M_PI / 180.0;
    double x_e = 0.0, y_n = 0.0;
    wgs84ToEnu(lat, lon, lat0_rad_, lon0_rad_, x_e, y_n);
    poly.push_back({x_e, y_n});
  }

  // Compute bounding box in ENU.
  double minx = poly[0].x, maxx = poly[0].x;
  double miny = poly[0].y, maxy = poly[0].y;
  for (const auto& p : poly) {
    if (p.x < minx) minx = p.x;
    if (p.x > maxx) maxx = p.x;
    if (p.y < miny) miny = p.y;
    if (p.y > maxy) maxy = p.y;
  }

  // Simple heuristic: sample candidate sweep angles and choose the one
  // minimizing number of lanes (width / spacing).
  double best_angle = 0.0;
  int best_lanes = std::numeric_limits<int>::max();

  for (int k = 0; k < angle_samples_; ++k) {
    const double theta = (M_PI * k) / angle_samples_;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    double min_proj = 1e9, max_proj = -1e9;
    for (const auto& p : poly) {
      const double xp = c * p.x + s * p.y;
      if (xp < min_proj) min_proj = xp;
      if (xp > max_proj) max_proj = xp;
    }
    const double width = max_proj - min_proj;
    const int lanes = static_cast<int>(std::ceil(width / lane_spacing_m_));
    if (lanes < best_lanes) {
      best_lanes = lanes;
      best_angle = theta;
    }
  }

  plan.sweep_angle_rad = best_angle;

  // Generate lanes with chosen sweep angle.
  const double c = std::cos(best_angle);
  const double s = std::sin(best_angle);

  double min_proj = 1e9, max_proj = -1e9;
  for (const auto& p : poly) {
    const double xp = c * p.x + s * p.y;
    if (xp < min_proj) min_proj = xp;
    if (xp > max_proj) max_proj = xp;
  }

  for (double d = min_proj; d <= max_proj; d += lane_spacing_m_) {
    // For each lane, march along its orthogonal direction and clip against
    // polygon bounds with point-in-polygon tests.
    const int samples = 50;
    EnuPoint first{}, last{};
    bool have_any = false;

    for (int i = 0; i <= samples; ++i) {
      const double t = static_cast<double>(i) / samples;
      // Parametric line in rotated frame.
      const double yp = miny + t * (maxy - miny);
      const double x_world = c * d - s * yp;
      const double y_world = s * d + c * yp;
      EnuPoint p{x_world, y_world};
      if (pointInPolygon(p, poly)) {
        if (!have_any) {
          first = p;
          have_any = true;
        }
        last = p;
      }
    }

    if (have_any) {
      plan.lanes.push_back({first, last});
    }
  }

  return plan;
}

}  // namespace awscr

