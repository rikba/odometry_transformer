#include "odometry_transformer/odometry_transformer.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace odometry_transformer {
OdometryTransformer::OdometryTransformer(const ros::NodeHandle &nh,
                                         const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), tf_listener_(tf_buffer_) {
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
    const Eigen::Affine3d T_ST =
        tf2::transformToEigen(tf_buffer_.lookupTransform(
            source_frame_, target_frame_, source_odometry->header.stamp));

    // Get source pose in inertial/world coordinate frame.
    Eigen::Affine3d T_IS;
    Eigen::fromMsg(source_odometry->pose.pose, T_IS);

    // Compute pose of target frame.
    const Eigen::Affine3d T_IT = T_IS * T_ST;

    // Compute linear twist of target frame.
    Eigen::Vector3d S_v_S, S_w_S;
    tf2::fromMsg(source_odometry->twist.twist.linear, S_v_S);
    tf2::fromMsg(source_odometry->twist.twist.angular, S_w_S);

    // Convert into inertial frame (not sure if this is necessary).
    const Eigen::Vector3d I_v_S = T_IS.rotation() * S_v_S;
    const Eigen::Vector3d I_w_S = T_IS.rotation() * S_w_S;
    const Eigen::Vector3d I_t_ST = T_IS.rotation() * T_ST.translation();

    // Rigid body velocity.
    const Eigen::Vector3d I_v_T = I_v_S + I_w_S.cross(I_t_ST);
    const Eigen::Vector3d T_v_T = T_IT.rotation().inverse() * I_v_T;

    // The angular velocity is identical anywhere on the rigid body.
    const Eigen::Vector3d T_w_T = T_ST.rotation().inverse() * S_w_S;

    // Convert to target odometry.
    nav_msgs::Odometry target_odometry;
    target_odometry.header = source_odometry->header;
    target_odometry.child_frame_id = target_frame_;
    target_odometry.pose.pose = Eigen::toMsg(T_IT);
    tf2::toMsg(T_v_T, target_odometry.twist.twist.linear);
    tf2::toMsg(T_w_T, target_odometry.twist.twist.angular);

    // Publish transformed odometry.
    odometry_pub_.publish(target_odometry);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}

} // namespace odometry_transformer
