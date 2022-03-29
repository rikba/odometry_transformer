#include "odometry_transformer/odometry_transformer.h"

namespace odometry_transformer {
OdometryTransformer::OdometryTransformer(const ros::NodeHandle &nh,
                                         const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  getRosParameters();
}

void OdometryTransformer::getRosParameters() {
  const std::string log_param_error = "Did not find ROS parameter: ";
  ROS_WARN_STREAM_COND(!nh_private_.getParam("parent_frame", parent_frame_),
                       log_param_error.c_str() << "parent_frame");
  ROS_WARN_STREAM_COND(!nh_private_.getParam("child_frame", child_frame_),
                       log_param_error.c_str() << "child_frame");
}

} // namespace odometry_transformer
