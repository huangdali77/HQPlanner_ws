#ifndef HQPLANNER_REFERENCE_LINE_H_
#define HQPLANNER_REFERENCE_LINE_H_
#include <cmath>
#include <utility>
#include <vector>

// #include "for_proto/config_param.h"

#include "hqplanner/for_proto/config_param.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/sl_boundary.h"
#include "hqplanner/math/box2d.h"
#include "hqplanner/math/cubic_spline.h"
#include "hqplanner/math/line_segment2d.h"
#include "hqplanner/math/vec2d.h"
namespace hqplanner {

class ReferenceLine {
 public:
  ReferenceLine() = default;
  ReferenceLine(const std::vector<double> &x, const std::vector<double> &y);
  explicit ReferenceLine(
      const std::vector<hqplanner::forproto::AnchorPoint> anchor_points);

  void AccumulateOnS();
  void ConstructReferenceLineByFixedStep();
  void ConstructReferenceLine2();
  double ComputeCurvature(double dx, double ddx, double dy, double ddy) const;
  double ComputeCurvatureDerivative(double dx, double ddx, double dddx,
                                    double dy, double ddy, double dddy) const;
  std::vector<hqplanner::forproto::ReferencePoint> GetReferenceLinePoints()
      const {
    return reference_line_points_;
  }

  double Length() const { return reference_line_points_.back().s; }

  double GetPositionXByS(double i_s) const;
  double GetPositionYByS(double i_s) const;
  double GetHeadingByS(double i_s) const;
  double GetKappaByS(double i_s) const;
  double GetKappaDerivativeByS(double i_s) const;
  // ReferencePoint GetReferencePoint(const double s) const;
  std::vector<hqplanner::forproto::AnchorPoint> GetAnchorPoints() {
    return anchor_points_;
  }

  bool SLToXY(const hqplanner::forproto::SLPoint &sl_point,
              hqplanner::math::Vec2d *const xy_point) const;
  bool XYToSL(const hqplanner::math::Vec2d &xy_point,
              hqplanner::forproto::SLPoint *const sl_point) const;

  hqplanner::forproto::ReferencePoint GetReferencePoint(const double x,
                                                        const double y) const;
  hqplanner::forproto::ReferencePoint GetReferencePoint(const double s) const;

  bool GetSLBoundary(const hqplanner::math::Box2d &box,
                     hqplanner::forproto::SLBoundary *const sl_boundary) const;

  bool GetApproximateSLBoundary(
      const hqplanner::math::Box2d &box,
      hqplanner::forproto::SLBoundary *const sl_boundary) const;
  bool GetLaneWidth(const double s, double *const lane_left_width,
                    double *const lane_right_width) const;
  bool Shrink(const hqplanner::math::Vec2d &point, double look_backward,
              double look_forward);
  double GetSpeedLimitFromS(const double s) const;
  void AddSpeedLimit(double start_s, double end_s, double speed_limit);

  bool IsOnRoad(const hqplanner::forproto::SLPoint &sl_point) const;
  bool IsOnRoad(const hqplanner::math::Vec2d &vec2d_point) const;
  template <class XYPoint>
  bool IsOnRoad(const XYPoint &xy) const {
    return IsOnRoad(hqplanner::math::Vec2d(xy.x(), xy.y()));
  }
  bool IsOnRoad(const hqplanner::forproto::SLBoundary &sl_boundary) const;

 private:
  std::vector<hqplanner::forproto::AnchorPoint> anchor_points_;
  std::vector<double> anchor_points_x_;
  std::vector<double> anchor_points_y_;
  std::vector<double> anchor_points_s_;
  hqplanner::math::CubicSpline anchor_points_x_s_;
  hqplanner::math::CubicSpline anchor_points_y_s_;
  //根据当前需要从长的参考线中shrink出一段，reference_points_需要在每一帧都要根据reference_line_points_更新
  std::vector<hqplanner::forproto::ReferencePoint> reference_line_points_;

  //根据上游给的anchor points生成的长的参考线，只有上游给的anchor
  // point有更新时route_points_才需要更新,也就是调用ConstructReferenceLineByFixedStep()函数
  std::vector<hqplanner::forproto::ReferencePoint> route_points_;
  std::vector<double> accumulated_s_;  // step = 0.1m

 private:
  struct SpeedLimit {
    double start_s = 0.0;
    double end_s = 0.0;
    double speed_limit = 0.0;  // unit m/s
    SpeedLimit() = default;
    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
  };
  std::vector<SpeedLimit> speed_limit_;
};

}  // namespace hqplanner

#endif