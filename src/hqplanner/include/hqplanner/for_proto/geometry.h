#ifndef HQPLANNER_FOR_PROTO_GEOMETRY_H_
#define HQPLANNER_FOR_PROTO_GEOMETRY_H_

#include <string>
#include <vector>
namespace hqplanner {
namespace forproto {

// A point in the map reference frame. The map defines an origin, whose
// coordinate is (0, 0, 0).
// Most modules, including localization, perception, and prediction, generate
// results based on the map reference frame.
// Currently, the map uses Universal Transverse Mercator (UTM) projection. See
// the link below for the definition of map origin.
//   https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
// The z field of PointENU can be omitted. If so, it is a 2D location and we do
// not care its height.
struct PointENU {
  double x;        //= 1 [default = nan];// East from the origin, in meters.
  double y;        //= 2 [default = nan]// North from the origin, in meters.
  double z = 0.0;  // Up from the WGS-84 ellipsoid, in
                   // meters.
};

// A point in the global reference frame. Similar to PointENU, PointLLH allows
// omitting the height field for representing a 2D location.
struct PointLLH {
  // Longitude in degrees, ranging from -180 to 180.
  double lon;  // = 1 [default = nan];
               // Latitude in degrees, ranging from -90 to 90.
  double lat;  //= 2 [default = nan];
               // WGS-84 ellipsoid height in meters.
  double height = 0.0;
};

// A general 2D point. Its meaning and units depend on context, and must be
// explained in comments.
struct Point2D {
  double x;  //= 1 [default = nan];
  double y;  //= 2 [default = nan];
};

// A general 3D point. Its meaning and units depend on context, and must be
// explained in comments.
struct Point3D {
  double x;  // = 1 [default = nan];
  double y;  //= 2 [default = nan];
  double z;  //= 3 [default = nan];
};

// A unit quaternion that represents a spatial rotation. See the link below for
// details.
//   https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
// The scalar part qw can be omitted. In this case, qw should be calculated by
//   qw = sqrt(1 - qx * qx - qy * qy - qz * qz).
struct Quaternion {
  double qx;  // = 1 [default = nan];
  double qy;  // = 2 [default = nan];
  double qz;  //= 3 [default = nan];
  double qw;  // = 4 [default = nan];
};

// A general polygon, points are counter clockwise
struct Polygon {
  std::vector<Point3D> point;
};

}  // namespace forproto
}  // namespace hqplanner

#endif