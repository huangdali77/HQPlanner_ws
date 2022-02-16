#ifndef HQPLANNER_TASKS_EM_EM_PLANNER_H_
#define HQPLANNER_TASKS_EM_EM_PLANNER_H_

#include <memory>
#include <string>
#include <vector>

#include "hqplanner/for_proto/planning_config.h"
#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/math/curve1d/quintic_polynomial_curve1d.h"
#include "hqplanner/path/path_data.h"
#include "hqplanner/reference_line/reference_line.h"
#include "hqplanner/reference_line/reference_line_info.h"
#include "hqplanner/tasks/em/planner.h"
#include "hqplanner/tasks/task.h"
namespace hqplanner {
namespace tasks {

/**
 * @class EMPlanner
 * @brief EMPlanner is an expectation maximization planner.
 */

class EMPlanner : public Planner {
 public:
  /**
   * @brief Constructor
   */
  EMPlanner() = default;

  /**
   * @brief Destructor
   */
  virtual ~EMPlanner() = default;

  bool Init(const hqplanner::forproto::PlanningConfig& config) override;
  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  bool Plan(const hqplanner::forproto::TrajectoryPoint& planning_init_point,
            Frame* frame) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  bool PlanOnReferenceLine(
      const hqplanner::forproto::TrajectoryPoint& planning_init_point,
      Frame* frame, ReferenceLineInfo* reference_line_info) override;

 private:
  //   void RegisterTasks();

  std::vector<hqplanner::forproto::SpeedPoint> GenerateInitSpeedProfile(
      const hqplanner::forproto::TrajectoryPoint& planning_init_point,
      const ReferenceLineInfo* reference_line_info);

  std::vector<hqplanner::forproto::SpeedPoint> DummyHotStart(
      const hqplanner::forproto::TrajectoryPoint& planning_init_point);

  std::vector<hqplanner::forproto::SpeedPoint> GenerateSpeedHotStart(
      const hqplanner::forproto::TrajectoryPoint& planning_init_point);

  void GenerateFallbackPathProfile(const ReferenceLineInfo* reference_line_info,
                                   hqplanner::path::PathData* path_data);

  void GenerateFallbackSpeedProfile(
      const ReferenceLineInfo* reference_line_info,
      hqplanner::speed::SpeedData* speed_data);

  hqplanner::speed::SpeedData GenerateStopProfile(const double init_speed,
                                                  const double init_acc) const;

  hqplanner::speed::SpeedData GenerateStopProfileFromPolynomial(
      const double init_speed, const double init_acc) const;

  bool IsValidProfile(
      const hqplanner::math::QuinticPolynomialCurve1d& curve) const;

  void RecordObstacleDebugInfo(ReferenceLineInfo* reference_line_info);

  void RecordDebugInfo(ReferenceLineInfo* reference_line_info,
                       const std::string& name, const double time_diff_ms);

  //   apollo::util::Factory<TaskType, Task> task_factory_;
  std::vector<std::unique_ptr<Task>> tasks_;
};

}  // namespace tasks
}  // namespace hqplanner

#endif  // MODULES_PLANNING_PLANNER_EM_EM_PLANNER_H_
