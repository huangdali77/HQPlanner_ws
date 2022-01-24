#include "hqplanner/reference_line/reference_line.h"

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "hqplanner/ref_line_test.h"

using hqplanner::ReferenceLine;
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ReferencePoint;
int main(int argc, char **argv) {
  ros::init(argc, argv, "ref_line_pub");
  ros::NodeHandle n;

  // defining Publisher
  ros::Publisher pub =
      n.advertise<hqplanner::ref_line_test>("/ref_line_points", 10);

  std::vector<double> x = {
      92.2679315918, 91.7368744032, 91.2058172146, 90.6747600259, 90.1217327927,
      89.3523723898, 88.4440988632, 87.4743322588, 86.5204926224, 85.66,
      84.8945767188, 84.1008630548, 83.2800137176, 82.4331834167, 81.5615268616,
      80.6661987617, 79.7483538266, 78.8091467658, 77.8497322887, 76.871265105,
      75.874899924,  74.8617914552, 73.8330944082, 72.7899634925, 71.7335534176,
      70.6650188929, 69.585514628,  68.4961953323, 67.3982157154, 66.2927304867,
      65.1808943558, 64.0638620322, 62.9427882253, 61.8188276447, 60.6931349998,
      59.5668650002, 58.4411723553, 57.3172117747, 56.1961379678, 55.0791056442,
      53.9672695133, 52.8617842846, 51.7638046677, 50.674485372,  49.5949811071,
      48.5264465824, 47.4700365075, 46.4269055918, 45.3982085448, 44.385100076,
      43.388734895,  42.4102677113, 41.4508532342, 40.5116461734, 39.5938012383,
      38.6984731384, 37.8268165833, 36.9799862824, 36.1591369452, 35.3654232812,
      34.6};
  std::vector<double> y = {
      -100.213542235, -101.060878215, -101.908214194, -102.755550174,
      -103.50964406,  -104.362357115, -105.205219287, -105.929760526,
      -106.42751078,  -106.59,        -106.549869862, -106.508225929,
      -106.465129988, -106.420643827, -106.374829234, -106.327747996,
      -106.2794619,   -106.230032734, -106.179522286, -106.127992344,
      -106.075504694, -106.022121124, -105.967903423, -105.912913377,
      -105.857212774, -105.800863401, -105.743927047, -105.686465499,
      -105.628540543, -105.570213969, -105.511547563, -105.452603113,
      -105.393442407, -105.334127232, -105.274719375, -105.215280625,
      -105.155872768, -105.096557593, -105.037396887, -104.978452437,
      -104.919786031, -104.861459457, -104.803534501, -104.746072953,
      -104.689136599, -104.632787226, -104.577086623, -104.522096577,
      -104.467878876, -104.414495306, -104.362007656, -104.310477714,
      -104.259967266, -104.2105381,   -104.162252004, -104.115170766,
      -104.069356173, -104.024870012, -103.981774071, -103.940130138,
      -103.9};
  ReferenceLine ref_line(x, y);
  std::vector<ReferencePoint> reference_line_points =
      ref_line.GetReferenceLinePoints();

  std::cout << reference_line_points.size() << std::endl;
  std::vector<AnchorPoint> anchor_points = ref_line.GetAnchorPoints();

  std::vector<geometry_msgs::Point> arrp;
  for (auto &ref_point : reference_line_points) {
    geometry_msgs::Point temp;
    temp.x = ref_point.x;
    temp.y = ref_point.y;
    temp.z = 0;
    arrp.push_back(temp);
  }
  hqplanner::ref_line_test result_ref_line;
  result_ref_line.path = arrp;

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    for (int i = 0; i < arrp.size(); ++i) {
      pub.publish(result_ref_line);
      ROS_INFO("%d: %f  %d: %f", i, result_ref_line.path[i].x, i,
               result_ref_line.path[i].y);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
