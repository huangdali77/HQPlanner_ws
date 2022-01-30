#ifndef HQPLANNER_FOR_PROTO_PNC_POINT_H_
#define HQPLANNER_FOR_PROTO_PNC_POINT_H_

#include <string>

namespace hqplanner {
namespace forproto {
struct PathPoint {
  // coordinates
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  // direction on the x-y plane
  double theta = 0.0;
  // curvature on the x-y planning
  double kappa = 0.0;
  // accumulated distance from beginning of the path
  double s = 0.0;

  // derivative of kappa w.r.t s.
  double dkappa = 0.0;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa = 0.0;
  // The lane ID where the path point is on
  std::string lane_id = "";
};

struct TrajectoryPoint {
  // path point
  PathPoint path_point;

  // linear velocity
  double v;  // in [m/s]
  double v_x;
  double v_y;
  // linear acceleration
  double a;
  double a_x;
  double a_y;
  // relative time from beginning of the trajectory
  double relative_time;
};

struct AnchorPoint {
 public:
  AnchorPoint() = default;
  AnchorPoint(double x, double y, double s)
      : cartesian_x(x), cartesian_y(y), frenet_s(s) {}
  // x-y plane
  double cartesian_x;
  double cartesian_y;
  // frenet plane
  double frenet_s;
  /* data */
};

struct ReferencePoint {
 public:
  ReferencePoint() = default;
  ReferencePoint(double xx, double yy, double ss) : x(xx), y(yy), s(ss) {}
  ReferencePoint(double ss) : s(ss) {}
  double x;
  double y;
  double s;
  // double yaw;
  // double curvature;
  // double d_curvature;
  // ===============
  double kappa = 0.0;
  double dkappa = 0.0;
  double heading = 0.0;
};

struct SLPoint {
  double s;
  double l;
};

struct FrenetFramePoint {
  double s;
  double l;
  double dl;
  double ddl;
};

struct SpeedPoint {
  double s;
  double t;
  // speed (m/s)
  double v;
  // acceleration (m/s^2)
  double a;
  // jerk (m/s^3)
  double da;
};

}  // namespace forproto
}  // namespace hqplanner

#endif