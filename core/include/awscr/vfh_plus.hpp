#pragma once

#include "types.hpp"

#include <vector>

namespace awscr {

// Vector Field Histogram+ obstacle avoidance operating on a planar LiDAR scan.
class VfhPlus {
 public:
  VfhPlus(double sector_width_rad, double safety_distance_m);

  // Returns modified desired heading (rad) in world frame.
  double compute_steering(double desired_heading_rad,
                          double current_heading_rad,
                          const LidarScan& scan);

 private:
  double sector_width_rad_;
  double safety_distance_m_;
  double last_selected_angle_rad_{0.0};
};

}  // namespace awscr

