#include "odometry_transformer/odometry_transformer.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace odometry_transformer {
OdometryTransformer::OdometryTransformer(const ros::NodeHandle &nh,
                                         const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private),
      tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_)) {
  getRosParameters();
  subscribeToRosTopics();
  advertiseRosTopics();
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

void OdometryTransformer::advertiseRosTopics() {
  odometry_pub_ =
      nh_.advertise<nav_msgs::Odometry>("target_odometry", queue_size_);
  ROS_INFO("Advertising %s", odometry_pub_.getTopic().c_str());
}

void OdometryTransformer::receiveOdometry(
    const nav_msgs::OdometryConstPtr &source_odometry) {
  ROS_INFO_ONCE("Received first odometry message.");

  try {
    // Get source to target calibration.
    auto tf_source_to_target =
        tf_buffer_.lookupTransform(source_frame_, target_frame_, ros::Time(0));

    // Setup target odometry message.
    nav_msgs::Odometry target_odometry;
    target_odometry.header = source_odometry->header;
    target_odometry.child_frame_id = target_frame_;

    // Tranform odometry pose.
    geometry_msgs::PoseStamped source_pose, target_pose;
    source_pose.header = source_odometry->header;
    source_pose.pose = source_odometry->pose.pose;
    target_pose.header = target_odometry.header;
    tf2::doTransform(source_pose, target_pose, tf_source_to_target);
    target_odometry.pose.pose = target_pose.pose;

    // Transform odometry twist.

    // Publish transformed odometry.
    odometry_pub_.publish(target_odometry);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

} // namespace odometry_transformer
