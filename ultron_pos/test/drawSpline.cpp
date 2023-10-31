#include "HermiteSpline.hpp"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "drawSpline");
  ros::NodeHandle nh;

  std::vector<KnotPoint> knots(6);
  knots[0].position << 0, 0, 0;
  knots[0].velocity << 0.1, 0, 0;
  knots[1].position << 0.2, 0, 0;
  knots[1].velocity << 1, 0, 0;
  knots[2].position << 1.9, 0.5, 1;
  knots[2].velocity << 0.1, 0, 0;
  knots[3].position << 2.1, 0.5, 1;
  knots[3].velocity << 0.1, 0, 0;
  knots[4].position << 3.8, 0, 0;
  knots[4].velocity << 1, 0, 0;
  knots[5].position << 4, 0, 0;
  knots[5].velocity << 0.1, 0, 0;

  HermiteSpline hermite_spline{knots, 10};
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PointStamped>("Point", 10);
  ros::Rate loop_rate(100);

  // Publish the spline once
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "odom";
  line_strip.header.stamp = ros::Time::now();
  // line_strip.ns = "drawSpline";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.01;
  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;
  for (double t = 0.0; t < 10.0; t += 0.01) {
    geometry_msgs::Point p;
    auto position = hermite_spline.getPosition(t);
    p.x = position(0);
    p.y = position(1);
    p.z = position(2);
    line_strip.points.push_back(p);
  }
  marker_pub.publish(line_strip);

  while (ros::ok()) {
    marker_pub.publish(line_strip);
    for(double t=0.0;t<10.0; t+=0.01){
      // Publish the current position
      geometry_msgs::PointStamped p;
      auto position = hermite_spline.getPosition(t);
      p.header.frame_id = "odom";
      p.header.stamp = ros::Time::now();
      p.point.x = position(0);
      p.point.y = position(1);
      p.point.z = position(2);

      pose_pub.publish(p);
      ros::spinOnce();
      loop_rate.sleep();

    }
    
  }
  return 0;
}