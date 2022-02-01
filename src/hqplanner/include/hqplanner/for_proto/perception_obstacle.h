#ifndef HQPLANNER_FOR_PROTO_PERCEPTION_OBSTACLE_H_
#define HQPLANNER_FOR_PROTO_PERCEPTION_OBSTACLE_H_

#include <memory>
#include <vector>
namespace hqplanner {
namespace forproto {
struct Point {
  double x = 0.0;  // in meters.
  double y = 0.0;  // in meters.
  double z = 0.0;  // height in meters.
};

struct PerceptionObstacle {
  std::int32_t id = 0;  // obstacle ID.
  Point position;       // obstacle position in the world coordinate
                        // system.
  double theta = 0.0;   // heading in the world coordinate system.
  Point velocity;       // obstacle velocity.

  // Size of obstacle bounding box.
  double length = 0.0;  // obstacle length.
  double width = 0.0;   // obstacle width.
  double height = 0.0;  // obstacle height.

  std::vector<Point> polygon_point;  // obstacle corner points.
  // duration of an obstacle since detection in s.
  double tracking_time = 0.0;

  enum Type {
    UNKNOWN = 0,
    UNKNOWN_MOVABLE = 1,
    UNKNOWN_UNMOVABLE = 2,  //静止障碍物
    PEDESTRIAN = 3,  // Pedestrian, usually determined by moving behaviour.
    BICYCLE = 4,     // bike, motor bike
    VEHICLE = 5      // Passenger car or truck.
  };
  Type type;               // obstacle type
  double timestamp = 0.0;  // GPS time in seconds.

  // Just for offline debuging, onboard will not fill this field.
  // Format like : [x0, y0, z0, x1, y1, z1...]
  std::vector<double> point_cloud;

  double confidence = 1.0;
  enum ConfidenceType {
    CONFIDENCE_UNKNOWN = 0,
    CONFIDENCE_CNN = 1,
    CONFIDENCE_RADAR = 2
  };
  ConfidenceType confidence_type = CONFIDENCE_CNN;
  std::vector<Point> drops;  // trajectory of object.
};

struct PerceptionObstacles {
  std::vector<PerceptionObstacle> perception_obstacle;  // An array of obstacles
  //   apollo.common.Header header = 2;                      // Header
  //   apollo.common.ErrorCode error_code = 3 [default = OK];
  //   LaneMarkers lane_marker = 4;
  //   CIPVInfo cipv_info = 5;  // closest in path vehicle
};

}  // namespace forproto
}  // namespace hqplanner
#endif