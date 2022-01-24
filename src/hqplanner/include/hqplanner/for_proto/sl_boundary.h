#ifndef HQPLANNER_FOR_PROTO_SL_BOUNDARY_H_
#define HQPLANNER_FOR_PROTO_SL_BOUNDARY_H_
namespace hqplanner {
namespace forproto {
/////////////////////////////////////////////////////////////////
// The start_s and end_s are longitudinal values.
// start_s <= end_s.
//
//              end_s
//                ^
//                |
//          S  direction
//                |
//            start_s
//
// The start_l and end_l are lateral values.
// start_l <= end_l. Left side of the reference line is positive,
// and right side of the reference line is negative.
//  end_l  <-----L direction---- start_l
/////////////////////////////////////////////////////////////////

struct SLBoundary {
  double start_s = 0.0;
  double end_s = 0.0;
  double start_l = 0.0;
  double end_l = 0.0;
};
}  // namespace forproto
}  // namespace hqplanner

#endif