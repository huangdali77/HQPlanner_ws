#include "hqplanner/path/path_data.h"

#include <assert.h>
#include <ros/ros.h>

#include "hqplanner/for_proto/pnc_point.h"
namespace hqplanner {
namespace path {
using hqplanner::forproto::FrenetFramePoint;
using hqplanner::forproto::PathPoint;
using hqplanner::forproto::ReferencePoint;
using hqplanner::forproto::SLPoint;
using hqplanner::math::CartesianFrenetConverter;
using hqplanner::math::Vec2d;

bool PathData::SetDiscretizedPath(const DiscretizedPath &path) {
  if (reference_line_ == nullptr) {
    // AERROR << "Should NOT set discretized path when reference line is
    // nullptr. "
    //           "Please set reference line first.";
    return false;
  }
  discretized_path_ = path;
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    // AERROR << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  assert(discretized_path_.NumOfPoints() == frenet_path_.points().size());
  // =============NoNeed=============
  // 保留历史规划的路径数目为3
  if (path_data_history_.size() > 3) {
    path_data_history_.pop_front();
  }
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

bool PathData::SetFrenetPath(const FrenetFramePath &frenet_path) {
  if (reference_line_ == nullptr) {
    ROS_INFO("Should NOT set frenet path when reference line is nullptr. ");

    return false;
  }
  frenet_path_ = frenet_path;
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    ROS_INFO("Fail to transfer frenet path to discretized path.");

    return false;
  }
  assert(discretized_path_.NumOfPoints() == frenet_path_.points().size());
  // =============NoNeed=============
  // 保留历史规划的路径数目为3
  if (path_data_history_.size() > 3) {
    path_data_history_.pop_front();
  }
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

const DiscretizedPath &PathData::discretized_path() const {
  return discretized_path_;
}

bool PathData::Empty() const {
  return discretized_path_.NumOfPoints() == 0 &&
         frenet_path_.NumOfPoints() == 0;
}

std::list<std::pair<DiscretizedPath, FrenetFramePath>>
    &PathData::path_data_history() {
  return path_data_history_;
}

const FrenetFramePath &PathData::frenet_frame_path() const {
  return frenet_path_;
}

void PathData::SetReferenceLine(const ReferenceLine *reference_line) {
  Clear();
  reference_line_ = reference_line;
}

bool PathData::GetPathPointWithPathS(const double s,
                                     PathPoint *const path_point) const {
  *path_point = discretized_path_.Evaluate(s);
  return true;
}

bool PathData::GetPathPointWithRefS(const double ref_s,
                                    PathPoint *const path_point) const {
  assert(reference_line_ != nullptr);
  assert(path_point != nullptr);
  assert(discretized_path_.path_points().size() ==
         frenet_path_.points().size());
  //   DCHECK_NOTNULL(reference_line_);
  //   DCHECK_NOTNULL(path_point);
  //   DCHECK_EQ(discretized_path_.path_points().size(),
  //             frenet_path_.points().size());
  if (ref_s < 0) {
    // AERROR << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }
  if (ref_s > frenet_path_.points().back().s) {
    // AERROR << "ref_s is larger than the length of frenet_path_ length ["
    //        << frenet_path_.points().back().s() << "].";
    return false;
  }

  std::uint32_t index = 0;
  const double kDistanceEpsilon = 1e-3;
  for (std::uint32_t i = 0; i + 1 < frenet_path_.points().size(); ++i) {
    if (fabs(ref_s - frenet_path_.points().at(i).s) < kDistanceEpsilon) {
      *path_point = discretized_path_.path_points().at(i);
      return true;
    }
    if (frenet_path_.points().at(i).s < ref_s &&
        ref_s <= frenet_path_.points().at(i + 1).s) {
      index = i;
      break;
    }
  }
  double r = (ref_s - frenet_path_.points().at(index).s) /
             (frenet_path_.points().at(index + 1).s -
              frenet_path_.points().at(index).s);

  const double discretized_path_s =
      discretized_path_.path_points().at(index).s +
      r * (discretized_path_.path_points().at(index + 1).s -
           discretized_path_.path_points().at(index).s);
  *path_point = discretized_path_.Evaluate(discretized_path_s);

  return true;
}

void PathData::Clear() {
  discretized_path_.Clear();
  frenet_path_.Clear();
  reference_line_ = nullptr;
}

// std::string PathData::DebugString() const {
//   const auto &path_points = discretized_path_.path_points();
//   const auto limit =
//       std::min(path_points.size(),
//                static_cast<size_t>(FLAGS_trajectory_point_num_for_debug));

//   return apollo::common::util::StrCat(
//       "[\n",
//       apollo::common::util::PrintDebugStringIter(
//           path_points.begin(), path_points.begin() + limit, ",\n"),
//       "]\n");
// }

bool PathData::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
  assert(discretized_path != nullptr);
  //   DCHECK_NOTNULL(discretized_path);
  std::vector<PathPoint> path_points;
  for (const FrenetFramePoint &frenet_point : frenet_path.points()) {
    SLPoint sl_point;
    Vec2d cartesian_point;
    sl_point.s = frenet_point.s;
    sl_point.l = frenet_point.l;
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      //   AERROR << "Fail to convert sl point to xy point";
      return false;
    }
    ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s);

    PathPoint path_point;
    path_point.x = cartesian_point.x();
    path_point.y = cartesian_point.y();
    path_point.z = 0;

    path_point.theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading, ref_point.kappa, frenet_point.l, frenet_point.dl);
    path_point.kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa, ref_point.dkappa, frenet_point.l, frenet_point.dl,
        frenet_point.ddl);

    path_point.dkappa = 0;
    path_point.ddkappa = 0;

    if (path_points.empty()) {
      //   frenet_path_中的s是frenet坐标系中的真实s坐标，
      //  discretized_path_中的s是是以局部规划得到的路径起点为原点的累计距离

      path_point.s = 0.0;
      path_point.dkappa = 0.0;
    } else {
      Vec2d last(path_points.back().x, path_points.back().y);
      Vec2d current(path_point.x, path_point.y);
      double distance = (last - current).Length();
      path_point.s = path_points.back().s + distance;
      path_point.dkappa =
          (path_point.kappa - path_points.back().kappa) / distance;
    }
    path_points.push_back(std::move(path_point));
  }
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

bool PathData::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
  assert(frenet_path != nullptr);
  assert(reference_line_ != nullptr);
  //   CHECK_NOTNULL(frenet_path);
  //   CHECK_NOTNULL(reference_line_);
  std::vector<FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->Length();
  for (const auto &path_point : discretized_path.path_points()) {
    SLPoint sl_point;
    if (!reference_line_->XYToSL(Vec2d(path_point.x, path_point.y),
                                 &sl_point)) {
      return false;
    }
    FrenetFramePoint frenet_point;
    // NOTICE: does not set dl and ddl here. Add if needed.
    frenet_point.s = std::max(0.0, std::min(sl_point.s, max_len));
    frenet_point.l = sl_point.l;
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}
}  // namespace path
}  // namespace hqplanner
