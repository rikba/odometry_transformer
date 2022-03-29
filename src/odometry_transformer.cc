#include "odometry_transformer/odometry_transformer.h"

namespace odometry_transformer {
OdometryTransformer::OdometryTransformer(const ros::NodeHandle &nh,
                                         const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  getRosParameters();
  subscribeToRosTopics();
}

void OdometryTransformer::getRosParameters() {
  const std::string log_param_error = "Did not find ROS parameter: ";

  ROS_WARN_STREAM_COND(!nh_private_.getParam("source_frame", source_frame_),
                       log_param_error.c_str() << "source_frame");
  ROS_WARN_STREAM_COND(!nh_private_.getParam("target_frame", target_frame_),
                       log_param_error.c_str() << "target_frame");
  ROS_INFO("Transforming odometry from %s frame to %s frame.",
           source_frame_.c_str(), target_frame_.c_str());

  nh_private_.getParam("queue_size", queue_size_);
  ROS_INFO("Odometry queue size: %d", queue_size_);
}

void OdometryTransformer::subscribeToRosTopics() {
  odometry_sub_ = nh_.subscribe("source_odometry", queue_size_,
                                &OdometryTransformer::receiveOdometry, this);
  ROS_INFO("Subscribed to %s", odometry_sub_.getTopic().c_str());
}

void OdometryTransformer::receiveOdometry(
    const nav_msgs::OdometryConstPtr &msg) {
  ROS_INFO_ONCE("Received first odometry message.");
}

} // namespace odometry_transformer
