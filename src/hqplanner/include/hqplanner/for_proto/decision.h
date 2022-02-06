#ifndef HQPLANNER_FOR_PROTO_DECISION_H_
#define HQPLANNER_FOR_PROTO_DECISION_H_
#include "hqplanner/for_proto/geometry.h"
namespace hqplanner {
namespace forproto {

enum StopReasonCode {
  STOP_REASON_HEAD_VEHICLE = 1,
  STOP_REASON_DESTINATION = 2,
  STOP_REASON_PEDESTRIAN = 3,
  STOP_REASON_OBSTACLE = 4,
  STOP_REASON_PREPARKING = 5,
  STOP_REASON_SIGNAL = 100,  // only for red signal
  STOP_REASON_STOP_SIGN = 101,
  STOP_REASON_YIELD_SIGN = 102,
  STOP_REASON_CLEAR_ZONE = 103,
  STOP_REASON_CROSSWALK = 104,
  STOP_REASON_CREEPER = 105,
  STOP_REASON_REFERENCE_END = 106,  // end of the reference_line
  STOP_REASON_YELLOW_SIGNAL = 107,  // yellow signal
  STOP_REASON_PULL_OVER = 108       // pull over
};

struct ObjectSidePass {
  enum Type { LEFT = 1, RIGHT = 2 };
  Type type;
};

struct ObjectNudge {
  enum Type {
    LEFT_NUDGE = 1,   // drive from the left side of the obstacle
    RIGHT_NUDGE = 2,  // drive from the right side of the obstacle
    NO_NUDGE = 3      // No nudge is set.
  };
  Type type = NO_NUDGE;
  // minimum lateral distance in meters. positive if type = LEFT_NUDGE
  // negative if type = RIGHT_NUDGE
  double distance_l = 0.0;
};

struct ObjectIgnore {};

struct ObjectStop {
  StopReasonCode reason_code;
  double distance_s = 0.0;  // in meters
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading = 0.0;
  std::vector<std::string> wait_for_obstacle;
};

struct ObjectYield {
  double distance_s = 0.0;  // minimum longitudinal distance in meters
  PointENU fence_point;
  double fence_heading = 0.0;
  double time_buffer = 0.0;  // minimum time buffer required after the obstacle
                             // reaches the intersect point.
};

struct ObjectFollow {
  double distance_s = 0.0;  // minimum longitudinal distance in meters
  PointENU fence_point;
  double fence_heading = 0.0;
};

struct ObjectOvertake {
  double distance_s = 0.0;  // minimum longitudinal distance in meters
  PointENU fence_point;
  double fence_heading = 0.0;
  double time_buffer = 0.0;  // minimum time buffer required before the obstacle
                             // reaches the intersect point.
};

struct ObjectDecisionType {
  enum ObjectTag {
    IGNORE = 1,
    STOP = 2,
    FOLLOW = 3,
    YIELD = 4,
    OVERTAKE = 5,
    NUDGE = 6,
    SIDEPASS = 7,
    AVOID = 8,
    OBJECT_TAG_NOT_SET = 9
  };

  bool has_sidepass() const { return object_tag == SIDEPASS; }
  bool has_nudge() const { return object_tag == NUDGE; }
  // 纵向决策
  bool has_ignore() const { return object_tag == IGNORE; }
  bool has_stop() const { return object_tag == STOP; }
  bool has_yield() const { return object_tag == YIELD; }
  bool has_follow() const { return object_tag == FOLLOW; }
  bool has_overtake() const { return object_tag == OVERTAKE; }

  ObjectTag object_tag = OBJECT_TAG_NOT_SET;
  ObjectIgnore ignore_;
  ObjectNudge nudge_;
  ObjectSidePass sidepass_;
  ObjectStop stop_;
  ObjectYield yield_;
  ObjectOvertake overtake_;
  ObjectFollow follow_;

  ObjectIgnore ignore() const { return ignore_; }
  ObjectNudge nudge() const { return nudge_; }
  ObjectSidePass sidepass() const { return sidepass_; }
  ObjectFollow follow() const { return follow_; }
  ObjectStop stop() const { return stop_; }
  ObjectYield yield() const { return yield_; }
  ObjectOvertake overtake() const { return overtake_; }
};

struct MainStop {
  StopReasonCode reason_code;
  std::string reason = "";
  // When stopped, the front center of vehicle should be at this point.
  PointENU stop_point;
  // When stopped, the heading of the vehicle should be stop_heading.
  double stop_heading = 0.0;
  // optional apollo.routing.ChangeLaneType change_lane_type = 5;
};

struct ObjectDecision {
  std::string id = "";
  std::int32_t perception_id = 0;
  ObjectDecisionType object_decision;
};
struct ObjectDecisions {
  std::vector<ObjectDecision> decision;
};

}  // namespace forproto
}  // namespace hqplanner
#endif