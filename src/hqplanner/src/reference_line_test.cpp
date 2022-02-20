#include "hqplanner/reference_line/reference_line.h"

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <iostream>
#include <vector>

#include "hqplanner/math/cubic_spline.h"
#include "hqplanner/math/cubic_spline_clamped.h"
#include "hqplanner/math/cubic_spline_start_clamped.h"
#include "hqplanner/math/curve1d/quartic_polynomial_curve1d_pro.h"
// #include "hqplanner/ref_line_test.h"

using hqplanner::ReferenceLine;
using hqplanner::forproto::AnchorPoint;
using hqplanner::forproto::ReferencePoint;
using hqplanner::math::CubicSpline;
using hqplanner::math::CubicSplineClamped;
using hqplanner::math::CubicSplineStartClamped;
using hqplanner::math::QuarticPolynomialCurve1dPro;
int main(int argc, char **argv) {
  ros::init(argc, argv, "ref_line_pub");
  ros::NodeHandle n;

  // defining Publisher
  ros::Publisher pub =
      n.advertise<visualization_msgs::Marker>("ref_line_points", 10);

  // std::vector<double> x = {
  //     92.2679315918, 91.7368744032, 91.2058172146, 90.6747600259, 90.1217327927,
  //     89.3523723898, 88.4440988632, 87.4743322588, 86.5204926224, 85.66,
  //     84.8945767188, 84.1008630548, 83.2800137176, 82.4331834167, 81.5615268616,
  //     80.6661987617, 79.7483538266, 78.8091467658, 77.8497322887, 76.871265105,
  //     75.874899924,  74.8617914552, 73.8330944082, 72.7899634925, 71.7335534176,
  //     70.6650188929, 69.585514628,  68.4961953323, 67.3982157154, 66.2927304867,
  //     65.1808943558, 64.0638620322, 62.9427882253, 61.8188276447, 60.6931349998,
  //     59.5668650002, 58.4411723553, 57.3172117747, 56.1961379678, 55.0791056442,
  //     53.9672695133, 52.8617842846, 51.7638046677, 50.674485372,  49.5949811071,
  //     48.5264465824, 47.4700365075, 46.4269055918, 45.3982085448, 44.385100076,
  //     43.388734895,  42.4102677113, 41.4508532342, 40.5116461734, 39.5938012383,
  //     38.6984731384, 37.8268165833, 36.9799862824, 36.1591369452, 35.3654232812,
  //     34.6};
  // std::vector<double> y = {
  //     -100.213542235, -101.060878215, -101.908214194, -102.755550174,
  //     -103.50964406,  -104.362357115, -105.205219287, -105.929760526,
  //     -106.42751078,  -106.59,        -106.549869862, -106.508225929,
  //     -106.465129988, -106.420643827, -106.374829234, -106.327747996,
  //     -106.2794619,   -106.230032734, -106.179522286, -106.127992344,
  //     -106.075504694, -106.022121124, -105.967903423, -105.912913377,
  //     -105.857212774, -105.800863401, -105.743927047, -105.686465499,
  //     -105.628540543, -105.570213969, -105.511547563, -105.452603113,
  //     -105.393442407, -105.334127232, -105.274719375, -105.215280625,
  //     -105.155872768, -105.096557593, -105.037396887, -104.978452437,
  //     -104.919786031, -104.861459457, -104.803534501, -104.746072953,
  //     -104.689136599, -104.632787226, -104.577086623, -104.522096577,
  //     -104.467878876, -104.414495306, -104.362007656, -104.310477714,
  //     -104.259967266, -104.2105381,   -104.162252004, -104.115170766,
  //     -104.069356173, -104.024870012, -103.981774071, -103.940130138,
  //     -103.9};

  std::vector<double> x = {0, 10, 20, 30, 40, 50, 60, 70, 80};
  std::vector<double> y = {0, 11.1, 19.6, 10.2, 26, 49.3, 30.2, 23.3, 99.9};

  // ReferenceLine ref_line(x, y);
  // std::vector<ReferencePoint> reference_line_points =
  //     ref_line.reference_points();

  // // std::cout << reference_line_points.size() << std::endl;
  // std::vector<AnchorPoint> an_points = ref_line.GetAnchorPoints();
  CubicSpline cubic(x, y);
  CubicSplineClamped cubic_clamped(x, y, 0.0, -0.5);
  CubicSplineStartClamped cubic_start_clamped(x, y, 0.0, 0.5);
  double x0 = 0.0;
  double dx0 = 5.0;
  double ddx0 = 0.0;
  double x1 = 100.0;
  double dx1 = 5.5;
  double param = 6.0;

  QuarticPolynomialCurve1dPro quartic_curve(x0, dx0, ddx0, x1, dx1, param);

  ros::Rate loop_rate(1);
  while (ros::ok()) {
    visualization_msgs::Marker anchor_points, line_strip, line_strip1,
        line_strip2, line_strip3;
    anchor_points.header.frame_id = line_strip.header.frame_id =
        line_strip1.header.frame_id = line_strip2.header.frame_id =
            line_strip3.header.frame_id = "MyFrame";
    anchor_points.header.stamp = line_strip.header.stamp =
        line_strip1.header.stamp = line_strip2.header.stamp =
            line_strip3.header.stamp = ros::Time::now();

    anchor_points.ns = line_strip.ns = line_strip1.ns = line_strip2.ns =
        line_strip3.ns = "ref_line";

    anchor_points.action = line_strip.action = line_strip1.action =
        line_strip2.action = line_strip3.action =
            visualization_msgs::Marker::ADD;
    anchor_points.pose.orientation.w = line_strip.pose.orientation.w =
        line_strip1.pose.orientation.w = line_strip2.pose.orientation.w =
            line_strip3.pose.orientation.w = 1.0;
    anchor_points.id = 0;

    line_strip.id = 2;
    line_strip1.id = 3;
    line_strip2.id = 4;
    line_strip3.id = 5;
    anchor_points.type = visualization_msgs::Marker::POINTS;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip1.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip2.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip3.type = visualization_msgs::Marker::LINE_STRIP;

    anchor_points.scale.x = 0.3;
    anchor_points.scale.y = 0.3;

    line_strip.scale.x = 0.1;
    line_strip1.scale.x = 0.1;
    line_strip2.scale.x = 0.1;
    line_strip3.scale.x = 0.1;
    anchor_points.color.g = 1.0;
    anchor_points.color.a = 1.0;

    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    line_strip1.color.b = 1.0;
    line_strip1.color.a = 1.0;
    line_strip2.color.b = 0.5;
    line_strip2.color.r = 0.5;
    line_strip2.color.a = 1.0;

    line_strip3.color.b = 1.0;
    line_strip3.color.a = 1.0;
    double t = 0.0;
    // while (t <= 80) {
    //   double s = cubic.GetSplinePointValue(t);
    //   double s1 = cubic_clamped.GetSplinePointValue(t);
    //   double s2 = cubic_start_clamped.GetSplinePointValue(t);
    //   geometry_msgs::Point temp, temp1, temp2;

    //   temp2.x = temp1.x = temp.x = t;
    //   temp.y = s;
    //   temp1.y = s1;
    //   temp2.y = s2;
    //   temp2.z = temp1.z = temp.z = 0;
    //   line_strip.points.push_back(temp);
    //   line_strip1.points.push_back(temp1);
    //   line_strip2.points.push_back(temp2);
    //   t += 0.1;
    // }

    while (t <= 20) {
      double x = quartic_curve.Evaluate(0, t);
      geometry_msgs::Point temp;

      temp.x = t;
      temp.y = x;
      temp.z = 0;
      line_strip3.points.push_back(temp);
      t += 0.1;
    }

    // for (int i = 0; i < x.size(); ++i) {
    //   geometry_msgs::Point temp;
    //   temp.x = x[i];
    //   temp.y = y[i];
    //   temp.z = 0;
    //   anchor_points.points.push_back(temp);
    // }
    geometry_msgs::Point temp1, temp2;
    temp1.x = 0.0;
    temp1.y = 0.0;
    temp1.z = 0;
    anchor_points.points.push_back(temp1);
    temp2.x = param;
    temp2.y = x1;
    temp2.z = 0;
    anchor_points.points.push_back(temp2);

    // for (int i = 0; i < 2; ++i) {
    //   geometry_msgs::Point temp;
    //   temp.x = x[i];
    //   temp.y = y[i];
    //   temp.z = 0;
    //   anchor_points.points.push_back(temp);
    // }

    pub.publish(anchor_points);
    pub.publish(line_strip3);

    ROS_INFO("X0:%f", quartic_curve.Evaluate(0, 0));
    ROS_INFO("dX0:%f", quartic_curve.Evaluate(1, 0));
    ROS_INFO("ddX0:%f", quartic_curve.Evaluate(2, 0));

    ROS_INFO("X0.3:%f", quartic_curve.Evaluate(0, 0.3 * param));
    ROS_INFO("dX0.3:%f", quartic_curve.Evaluate(1, 0.3 * param));
    ROS_INFO("ddX0.3:%f", quartic_curve.Evaluate(2, 0.3 * param));

    ROS_INFO("X0.6:%f", quartic_curve.Evaluate(0, 0.6 * param));
    ROS_INFO("dX0.6:%f", quartic_curve.Evaluate(1, 0.6 * param));
    ROS_INFO("ddX0.6:%f", quartic_curve.Evaluate(2, 0.6 * param));
    ROS_INFO("X1:%f", quartic_curve.Evaluate(0, param));
    ROS_INFO("dX1:%f", quartic_curve.Evaluate(1, param));
    ROS_INFO("ddX1:%f", quartic_curve.Evaluate(2, param));
    // pub.publish(line_strip);
    // pub.publish(line_strip2);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
