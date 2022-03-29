#include <ros/ros.h>

#include "odometry_transformer/odometry_transformer.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_transformer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  auto ot = odometry_transformer::OdometryTransformer(nh, nh_private);
  ROS_INFO("Hello world!");
  ros::spin();
  return 0;
}
