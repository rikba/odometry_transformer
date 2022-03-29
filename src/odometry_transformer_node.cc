#include <ros/ros.h>

#include "odometry_transformer/odometry_transformer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_transformer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::spin();
  return 0;
}
