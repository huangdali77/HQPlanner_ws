#include "hqplanner/tasks/dp_st_speed/dp_st_graph.h"

#include <assert.h>
#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "hqplanner/for_proto/pnc_point.h"
#include "hqplanner/math/vec2d.h"
#include "hqplanner/speed/st_boundary.h"

namespace hqplanner {
namespace tasks {

using hqplanner::PathObstacle;
using hqplanner::forproto::ConfigParam;
using hqplanner::forproto::DpStSpeedConfig;
using hqplanner::forproto::SLBoundary;
using hqplanner::forproto::SpeedPoint;
using hqplanner::forproto::TrajectoryPoint;
using hqplanner::forproto::VehicleParam;
using hqplanner::math::Vec2d;
using hqplanner::speed::SpeedData;
using hqplanner::speed::StBoundary;
using hqplanner::speed::STPoint;
namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();

bool CheckOverlapOnDpStGraph(const std::vector<const StBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  const hqplanner::math::LineSegment2d seg(p1.point(), p2.point());
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    if (boundary->HasOverlap(seg)) {
      return true;
    }
  }
  return false;
}
}  // namespace

DpStGraph::DpStGraph(const StGraphData& st_graph_data,
                     const DpStSpeedConfig& dp_config,
                     const std::vector<const PathObstacle*>& obstacles,
                     const TrajectoryPoint& init_point,
                     const SLBoundary& adc_sl_boundary)
    : st_graph_data_(st_graph_data),
      dp_st_speed_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, obstacles, init_point_),
      adc_sl_boundary_(adc_sl_boundary) {
  // 速度规划的总路径长度为min(149, path_data.discretized_path().Length())
  dp_st_speed_config_.total_path_length = std::fmin(
      dp_st_speed_config_.total_path_length, st_graph_data_.path_data_length());
  // unit_s_=(0, 1]
  unit_s_ = dp_st_speed_config_.total_path_length /
            (dp_st_speed_config_.matrix_dimension_s - 1);
  // unit_t_=1
  unit_t_ = dp_st_speed_config_.total_time /
            (dp_st_speed_config_.matrix_dimension_t - 1);
}

bool DpStGraph::Search(SpeedData* const speed_data) {
  // 如果adc规划起点被障碍物阻塞，则停车
  constexpr float kBounadryEpsilon = 1e-2;
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      std::vector<SpeedPoint> speed_profile;
      float t = 0.0;
      for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t;
           ++i, t += unit_t_) {
        SpeedPoint speed_point;
        speed_point.s = 0.0;
        speed_point.t = t;

        speed_profile.emplace_back(speed_point);
      }
      speed_data->set_speed_vector(speed_profile);
      return true;
    }
  }

  // 当没有obstacle st boundary的时候速度为1m/s????????

  // if (st_graph_data_.st_boundaries().empty()) {
  //   ROS_INFO("No path obstacles, dp_st_graph output default speed profile.");

  //   std::vector<SpeedPoint> speed_profile;
  //   float s = 0.0;
  //   float t = 0.0;
  //   for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t &&
  //                   i < dp_st_speed_config_.matrix_dimension_s;
  //        ++i, t += unit_t_, s += unit_s_) {
  //     SpeedPoint speed_point;
  //     speed_point.s = s;
  //     speed_point.t = t;
  //     const float v_default = unit_s_ / unit_t_;
  //     speed_point.v = v_default;
  //     speed_point.a = 0.0;
  //     speed_profile.emplace_back(std::move(speed_point));
  //   }
  //   speed_data->set_speed_vector(std::move(speed_profile));
  //   return true;
  // }

  if (!InitCostTable()) {
    ROS_INFO("Initialize cost table failed.");
    assert(0);
    return false;
  }

  if (!CalculateTotalCost()) {
    ROS_INFO("Calculate total cost failed.");
    assert(0);
    return false;
  }

  if (!RetrieveSpeedProfile(speed_data)) {
    ROS_INFO("Retrieve best speed profile failed.");
    assert(0);
    return false;
  }
  return true;
}

bool DpStGraph::InitCostTable() {
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s;
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t;
  assert(dim_s > 2);
  assert(dim_t > 2);

  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  float curr_t = 0.0;
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
    auto& cost_table_i = cost_table_[i];
    float curr_s = 0.0;
    for (uint32_t j = 0; j < cost_table_i.size(); ++j, curr_s += unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }
  return true;
}

bool DpStGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row
  uint32_t next_highest_row = 0;
  uint32_t next_lowest_row = 0;

  for (size_t c = 0; c < cost_table_.size(); ++c) {
    int highest_row = 0;
    int lowest_row = cost_table_.back().size() - 1;

    // 计算当前列可用范围内的cost
    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      //   if (FLAGS_enable_multi_thread_in_dp_st_graph) {
      //     PlanningThreadPool::instance()->Push(
      //         std::bind(&DpStGraph::CalculateCostAt, this, c, r));
      //   } else {
      //     CalculateCostAt(c, r);
      //   }
      CalculateCostAt(c, r);
    }
    // if (FLAGS_enable_multi_thread_in_dp_st_graph) {
    //   PlanningThreadPool::instance()->Synchronize();
    // }

    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<float>::infinity()) {
        int h_r = 0;
        int l_r = 0;
        // 使用最大加减速度向后扩展
        GetRowRange(cost_cr, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return true;
}

void DpStGraph::GetRowRange(const StGraphPoint& point, int* next_highest_row,
                            int* next_lowest_row) {
  float v0 = 0.0;
  if (!point.pre_point()) {
    v0 = init_point_.v;
  } else {
    v0 = (point.index_s() - point.pre_point()->index_s()) * unit_s_ / unit_t_;
  }

  const int max_s_size = cost_table_.back().size() - 1;

  const float speed_coeff = unit_t_ * unit_t_;

  const float delta_s_upper_bound =
      v0 * unit_t_ + vehicle_param_.max_acceleration * speed_coeff;
  *next_highest_row =
      point.index_s() + static_cast<int>(delta_s_upper_bound / unit_s_);
  if (*next_highest_row >= max_s_size) {
    *next_highest_row = max_s_size;
  }

  const float delta_s_lower_bound = std::fmax(
      0.0, v0 * unit_t_ + vehicle_param_.max_deceleration * speed_coeff);
  *next_lowest_row =
      point.index_s() - static_cast<int>(delta_s_lower_bound / unit_s_);
  if (*next_lowest_row > max_s_size) {
    *next_lowest_row = max_s_size;
  } else if (*next_lowest_row < 0) {
    *next_lowest_row = 0;
  }
}

void DpStGraph::CalculateCostAt(const uint32_t c, const uint32_t r) {
  auto& cost_cr = cost_table_[c][r];
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr));
  if (cost_cr.obstacle_cost() > std::numeric_limits<float>::max()) {
    // 与障碍物的stboundary相交
    return;
  }

  const auto& cost_init = cost_table_[0][0];
  if (c == 0) {
    assert(r == 0);
    // DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    return;
  }

  float speed_limit =
      st_graph_data_.speed_limit().GetSpeedLimitByS(unit_s_ * r);
  if (c == 1) {
    // at = v1-v2
    const float acc = (r * unit_s_ / unit_t_ - init_point_.v) / unit_t_;
    // const float acc = 2 * (r * unit_s_ / unit_t_ - init_point_.v) / unit_t_;
    if (acc < dp_st_speed_config_.max_deceleration ||
        acc > dp_st_speed_config_.max_acceleration) {
      // totalcost默认的是infinite
      return;
    }

    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      return;
    }
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_init.total_cost() +
                         CalculateEdgeCostForSecondCol(r, speed_limit));
    cost_cr.SetPrePoint(cost_init);
    return;
  }

  constexpr float kSpeedRangeBuffer = 0.20;
  const uint32_t max_s_diff = static_cast<uint32_t>(
      ConfigParam::instance()->FLAGS_planning_upper_speed_limit *
      (1 + kSpeedRangeBuffer) * unit_t_ / unit_s_);
  const uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);

  const auto& pre_col = cost_table_[c - 1];

  if (c == 2) {
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
      // acc=(r * unit_s_ -  r_pre * unit_s_)/unit_t_- (r_pre * unit_s_)/unit_t_
      const float acc =
          (r * unit_s_ - 2 * r_pre * unit_s_) / (unit_t_ * unit_t_);
      if (acc < dp_st_speed_config_.max_deceleration ||
          acc > dp_st_speed_config_.max_acceleration) {
        continue;
      }

      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }

      const float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +
                         CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);

      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
      }
    }
    return;
  }
  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }

    const float curr_a = (cost_cr.index_s() * unit_s_ +
                          pre_col[r_pre].pre_point()->index_s() * unit_s_ -
                          2 * pre_col[r_pre].index_s() * unit_s_) /
                         (unit_t_ * unit_t_);
    if (curr_a > vehicle_param_.max_acceleration ||
        curr_a < vehicle_param_.max_deceleration) {
      continue;
    }
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }

    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }

    if (!prepre_graph_point.pre_point()) {
      continue;
    }
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();
    float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +
                 CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                   curr_point, speed_limit);

    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
    }
  }
}

bool DpStGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  float min_cost = std::numeric_limits<float>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  // 结束时间相同，距离不同
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }
  // 结束距离相同，结束时间不同
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr) {
    ROS_INFO("Fail to find the best feasible trajectory.");
    assert(0);
    return false;
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  while (cur_point != nullptr) {
    SpeedPoint speed_point;
    speed_point.s = cur_point->point().s();
    speed_point.t = cur_point->point().t();

    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  std::reverse(speed_profile.begin(), speed_profile.end());

  constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
  if (speed_profile.front().t > kEpsilon ||
      speed_profile.front().s > kEpsilon) {
    ROS_INFO("Fail to retrieve speed profile.");
    assert(0);
    return false;
  }
  speed_data->set_speed_vector(speed_profile);
  return true;
}

float DpStGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second,
                                   const STPoint& third, const STPoint& forth,
                                   const float speed_limit) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

float DpStGraph::CalculateEdgeCostForSecondCol(const uint32_t row,
                                               const float speed_limit) {
  float init_speed = init_point_.v;
  float init_acc = init_point_.a;
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

float DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                              const uint32_t pre_row,
                                              const float speed_limit) {
  float init_speed = init_point_.v;
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace tasks
}  // namespace hqplanner
