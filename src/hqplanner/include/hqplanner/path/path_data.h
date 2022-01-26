#ifndef HQPLANNER_PATH_PATH_DATA_H_
#define HQPLANNER_PATH_PATH_DATA_H_

#include <assert.h>

#include <algorithm>
#include <limits>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include "hqplanner/math/cartesian_frenet_conversion.h"
#include "hqplanner/path/discretized_path.h"
#include "hqplanner/path/frenet_frame_path.h"
#include "hqplanner/reference_line/reference_line.h"
namespace hqplanner {
namespace path {

class PathData {
 public:
  PathData() = default;

  bool SetDiscretizedPath(const DiscretizedPath &path);

  bool SetFrenetPath(const FrenetFramePath &frenet_path);

  void SetReferenceLine(const ReferenceLine *reference_line);

  const DiscretizedPath &discretized_path() const;

  const FrenetFramePath &frenet_frame_path() const;

  bool GetPathPointWithPathS(
      const double s, hqplanner::forproto::PathPoint *const path_point) const;

  std::list<std::pair<DiscretizedPath, FrenetFramePath>> &path_data_history();

  /*
   * brief: this function will find the path_point in discretized_path whose
   * projection to reference line has s value closest to ref_s.
   */
  bool GetPathPointWithRefS(
      const double ref_s,
      hqplanner::forproto::PathPoint *const path_point) const;

  void Clear();

  bool Empty() const;

  //   std::string DebugString() const;

 private:
  /*
   * convert frenet path to cartesian path by reference line
   */
  bool SLToXY(const FrenetFramePath &frenet_path,
              DiscretizedPath *const discretized_path);
  bool XYToSL(const DiscretizedPath &discretized_path,
              FrenetFramePath *const frenet_path);
  const ReferenceLine *reference_line_ = nullptr;
  DiscretizedPath discretized_path_;
  FrenetFramePath frenet_path_;
  std::list<std::pair<DiscretizedPath, FrenetFramePath>> path_data_history_;
};

}  // namespace path
}  // namespace hqplanner

#endif  // MODULES_PLANNING_COMMON_PATH_PATH_DATA_H_
