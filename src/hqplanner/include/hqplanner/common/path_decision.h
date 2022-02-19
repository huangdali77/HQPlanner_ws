#ifndef HQPLANNER_PATH_DECISION_H_
#define HQPLANNER_PATH_DECISION_H_

#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hqplanner/common/obstacle.h"
#include "hqplanner/common/path_obstacle.h"
#include "hqplanner/for_proto/decision.h"
#include "hqplanner/for_proto/geometry.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/for_proto/sl_boundary.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/speed/st_boundary.h"
namespace hqplanner {

class PathDecision {
 public:
  PathDecision() = default;

  // void AddPathObstacle(PathObstacle path_obstacle);
  PathObstacle *AddPathObstacle(const PathObstacle &path_obstacle);

  const std::map<std::string, std::shared_ptr<PathObstacle>> &path_obstacles()
      const;

  // PathObstacle Find(const std::string &object_id);
  bool AddLateralDecision(
      const std::string &tag, const std::string &object_id,
      const hqplanner::forproto::ObjectDecisionType &decision);
  bool AddLongitudinalDecision(
      const std::string &tag, const std::string &object_id,
      const hqplanner::forproto::ObjectDecisionType &decision);
  void SetStBoundary(const std::string &id,
                     const hqplanner::speed::StBoundary &boundary);
  void EraseStBoundaries();
  hqplanner::forproto::MainStop main_stop() const { return main_stop_; }
  const PathObstacle *Find(const std::string &object_id) const;

  PathObstacle *Find(const std::string &object_id);

  double stop_reference_line_s() const { return stop_reference_line_s_; }
  bool MergeWithMainStop(
      const hqplanner::forproto::ObjectStop &obj_stop,
      const std::string &obj_id, const ReferenceLine &ref_line,
      const hqplanner::forproto::SLBoundary &adc_sl_boundary);
  const std::vector<const PathObstacle *> &path_obstacle_items() const;

 private:
  std::map<std::string, std::shared_ptr<PathObstacle>> path_obstacles_;
  std::vector<const PathObstacle *> path_obstacle_items_;
  //   std::vector<const PathObstacle *> path_obstacle_items_;
  double stop_reference_line_s_ = std::numeric_limits<double>::max();
  hqplanner::forproto::MainStop main_stop_;
};

}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_PATH_DECISION_H_
