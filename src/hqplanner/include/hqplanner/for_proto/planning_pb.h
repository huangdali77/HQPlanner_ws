// #ifndef HQPLANNER_FOR_PROTO_PLANNING_PB_H_
// #define HQPLANNER_FOR_PROTO_PLANNING_PB_H_
// #include <vector>

// #include "hqplanner/for_proto/decision.h"
// #include "hqplanner/for_proto/geometry.h"
// #include "hqplanner/for_proto/pnc_point.h"

// // import "modules/canbus/proto/chassis.proto";
// // import "modules/common/proto/drive_state.proto";
// // import "modules/common/proto/geometry.proto";
// // import "modules/common/proto/header.proto";
// // import "modules/common/proto/pnc_point.proto";
// // import "modules/common/proto/vehicle_signal.proto";
// // import "modules/map/proto/map_id.proto";
// // import "modules/planning/proto/decision.proto";
// // import "modules/planning/proto/planning_internal.proto";

// namespace hqplanner {
// namespace forproto {

// // Deprecated: replaced by apollo.common.TrajectoryPoint
// struct ADCTrajectoryPoint {
//    double x = 0.0;  // in meters.
//    double y = 0.0;  // in meters.
//    double z = 0.0;  // height in meters.

//    double speed = 0.0;           // speed, in meters / second
//    double acceleration_s = 0.0;  // acceleration in s direction
//    double curvature = 0.0;       // curvature (k = 1/r), unit: (1/meters)
//   // change of curvature in unit s (dk/ds)
//    double curvature_change_rate = 0.0;
//   // in seconds (relative_time = time_of_this_state - timestamp_in_header)
//    double relative_time = 0.0;
//    double theta = 11;  // relative to absolute coordinate system
//   // calculated from the first point in this trajectory
//    double accumulated_s = 12;

//   // in meters, reference to route SL-coordinate
//    double s = 4 [deprecated = true];
//   // in meters, reference to route SL-coordinate
//    double l = 5 [deprecated = true];
// };

// // Deprecated: replaced by apollo.common.PathPoint
// message ADCPathPoint {
//    double x = 1;          // in meters
//    double y = 2;          // in meters
//    double z = 3;          // in meters
//    double curvature = 4;  // curvature (k = 1/r), unit: (1/meters)
//    double heading = 5;    // relative to absolute coordinate system
// }

// message ADCSignals {
//   enum SignalType {
//     LEFT_TURN = 1; RIGHT_TURN = 2; LOW_BEAM_LIGHT = 3; HIGH_BEAM_LIGHT = 4;
//     FOG_LIGHT = 5;
//     EMERGENCY_LIGHT = 6;
//   }
//   repeated SignalType signal = 1;
// }

// message EStop {
//   // is_estop == true when emergency stop is required
//    bool is_estop = 1;
//    string reason = 2;
// }

// message TaskStats {
//    string name = 1;
//    double time_ms = 2;
// }

// message LatencyStats {
//    double total_time_ms = 1;
//   repeated TaskStats task_stats = 2;
//    double init_frame_time_ms = 3;
// }

// // next id: 21
// message ADCTrajectory {
//    apollo.common.Header header = 1;

//    double total_path_length = 2;  // in meters
//    double total_path_time = 3;    // in seconds

//   // path data + speed data
//   repeated apollo.common.TrajectoryPoint trajectory_point = 12;

//    EStop estop = 6;

//   // path point without speed info
//   repeated apollo.common.PathPoint path_point = 13;

//   // is_replan == true mean replan triggered
//    bool is_replan = 9 [default = false];

//   // Specify trajectory gear
//    apollo.canbus.Chassis.GearPosition gear = 10;

//    apollo.planning.DecisionResult decision = 14;

//    LatencyStats latency_stats = 15;

//   // the routing used for current planning result
//    apollo.common.Header routing_header = 16;
//    apollo.planning_internal.Debug debug = 8;

//   // replaced by path_point
//   repeated ADCPathPoint adc_path_point = 7 [deprecated = true];
//   // replaced by trajectory_point
//   repeated ADCTrajectoryPoint adc_trajectory_point = 4 [deprecated = true];
//    apollo.common.VehicleSignal signal = 11 [deprecated = true];
//   enum RightOfWayStatus { UNPROTECTED = 0; PROTECTED = 1; }
//    RightOfWayStatus right_of_way_status = 17;

//   // lane id along reference line
//   repeated apollo.hdmap.Id lane_id = 18;

//   // set the engage advice for based on current planning result.
//    apollo.common.EngageAdvice engage_advice = 19;

//   // the region where planning cares most
//   message CriticalRegion { repeated apollo.common.Polygon region = 1; }

//   // critial region will be empty when planning is NOT sure which region is
//   // critical critial regions may or may not overlap
//    CriticalRegion critical_region = 20;
// }

// }  // namespace forproto
// }  // namespace hqplanner

// #endif