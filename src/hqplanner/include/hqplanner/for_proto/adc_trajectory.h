#ifndef HQPLANNER_FOR_PROTO_ADC_TRAJECTORY_H_
#define HQPLANNER_FOR_PROTO_ADC_TRAJECTORY_H_
#include <string>
#include <vector>

#include "hqplanner/for_proto/pnc_point.h"
namespace hqplanner {
namespace forproto {
struct EStop {
  // is_estop == true when emergency stop is required
  bool is_estop = false;
  std::string reason;
};
struct ADCPathPoint {
  double x;          // in meters
  double y;          // in meters
  double z;          // in meters
  double curvature;  // curvature (k = 1/r), unit: (1/meters)
  double heading;    // relative to absolute coordinate system
};

struct ADCTrajectoryPoint {
  double x;  // in meters.
  double y;  // in meters.
  double z;  // height in meters.

  double speed;           // speed, in meters / second
  double acceleration_s;  // acceleration in s direction
  double curvature;       // curvature (k = 1/r), unit: (1/meters)
                          // change of curvature in unit s (dk/ds)
  double curvature_change_rate;
  // in seconds (relative_time = time_of_this_state - timestamp_in_header)
  double relative_time;
  double theta;  // relative to absolute coordinate system
                 // calculated from the first point in this trajectory
  double accumulated_s;

  // in meters, reference to route SL-coordinate
  double s;
  // in meters, reference to route SL-coordinate
  double l;
};

struct ADCTrajectory {
  double header_time = 0.0;
  double total_path_length;  // in meters
  double total_path_time;    // in seconds

  // path data + speed data
  // planning.cpp中的trajectory_pb的信息在这里面
  std::vector<TrajectoryPoint> trajectory_point;

  EStop estop;

  // path point without speed info
  std::vector<PathPoint> path_point;

  // is_replan == true mean replan triggered
  bool is_replan = false;

  // replaced by path_point
  std::vector<ADCPathPoint> adc_path_point;

  // replaced by trajectory_point
  std::vector<ADCTrajectoryPoint> adc_trajectory_point;
};

}  // namespace forproto
}  // namespace hqplanner
#endif