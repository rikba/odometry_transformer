#ifndef ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_
#define ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_

#include <string>

#include <ros/ros.h>

namespace odometry_transformer {
class OdometryTransformer {
public:
  OdometryTransformer(const ros::NodeHandle &nh,
                      const ros::NodeHandle &nh_private);
  void getRosParameters();

private:
  std::string parent_frame_ = "";
  std::string child_frame_ = "";

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
};
} // namespace odometry_transformer

#endif // ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_
