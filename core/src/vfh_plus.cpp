#include "awscr/vfh_plus.hpp"

#include <algorithm>
#include <cmath>

namespace awscr {

VfhPlus::VfhPlus(double sector_width_rad, double safety_distance_m)
    : sector_width_rad_(sector_width_rad),
      safety_distance_m_(safety_distance_m) {}

double VfhPlus::compute_steering(double desired_heading_rad,
                                 double current_heading_rad,
                                 const LidarScan& scan) {
  if (scan.ranges_m.empty()) return desired_heading_rad;

  const double span = scan.angle_max_rad - scan.angle_min_rad;
  const int num_sectors =
      std::max(1, static_cast<int>(std::ceil(span / sector_width_rad_)));

  std::vector<double> histogram(num_sectors, 0.0);

  for (std::size_t i = 0; i < scan.ranges_m.size(); ++i) {
    const double r = scan.ranges_m[i];
    if (r <= 0.01) continue;
    const double angle = scan.angle_min_rad + i * scan.angle_step_rad;
    const int k = std::clamp(
        static_cast<int>((angle - scan.angle_min_rad) / sector_width_rad_),
        0, num_sectors - 1);
    const double weight =
        (r < safety_distance_m_) ? (1.0 / (r * r)) : 0.0;
    histogram[k] += weight;
  }

  // Threshold to binary free/occupied.
  std::vector<int> binary(num_sectors, 0);
  const double T = 1.0;
  for (int k = 0; k < num_sectors; ++k) {
    binary[k] = (histogram[k] > T) ? 1 : 0;
  }

  // Desired direction in LiDAR frame.
  const double rel_desired = desired_heading_rad - current_heading_rad;

  int best_sector = -1;
  double best_cost = 1e9;
  for (int k = 0; k < num_sectors; ++k) {
    if (binary[k] != 0) continue;
    const double center_angle =
        scan.angle_min_rad + (k + 0.5) * sector_width_rad_;
    const double cost =
        std::fabs(center_angle - rel_desired) +
        0.5 * std::fabs(center_angle - last_selected_angle_rad_);
    if (cost < best_cost) {
      best_cost = cost;
      best_sector = k;
      last_selected_angle_rad_ = center_angle;
    }
  }

  if (best_sector < 0) {
    // No free direction found: keep current heading.
    return current_heading_rad;
  }

  const double chosen_rel =
      scan.angle_min_rad + (best_sector + 0.5) * sector_width_rad_;
  return current_heading_rad + chosen_rel;
}

}  // namespace awscr

