#include "odometry_transformer/odometry_transformer.h"

#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace odometry_transformer {
OdometryTransformer::OdometryTransformer(const ros::NodeHandle &nh,
                                         const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
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

  // Decide if you should get sensor calibration from TF or parameters.
  bool lookup_tf = false;
  if (nh_private_.getParam("lookup_tf", lookup_tf) && lookup_tf) {
    ROS_INFO("Receiving T_ST_ from tf2 buffer.");
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try {
      T_ST_ = tf2::transformToEigen(tf_buffer.lookupTransform(
          source_frame_, target_frame_, ros::Time::now(), ros::Duration(1.0)));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Cannot lookup transform: %s", ex.what());
    }
  } else {
    ROS_INFO("Receiving T_ST_ from ROS parameter server.");
    std::vector<double> T_r_TS, q_TS;
    Eigen::Affine3d T_TS = Eigen::Affine3d::Identity();
    // First rotation, then translation!
    if (nh_private_.getParam("q_TS", q_TS) && q_TS.size() == 4) {
      // In Eigen the imaginary coefficient w is leading.
      T_TS *= Eigen::Quaterniond(q_TS[3], q_TS[0], q_TS[1], q_TS[2]);
    } else {
      ROS_WARN_STREAM(log_param_error.c_str() << "q_TS");
    }
    if (nh_private_.getParam("T_r_TS", T_r_TS) && T_r_TS.size() == 3) {
      T_TS *= Eigen::Translation3d(Eigen::Vector3d(T_r_TS.data()));
    } else {
      ROS_WARN_STREAM(log_param_error.c_str() << "T_r_TS");
    }
    T_ST_ = T_TS.inverse();

    // Broadcast TF if calibration is coming from parameter server.
    tf_static_br_.emplace();
    broadcastCalibration();
  }
  ROS_INFO_STREAM(
      "T_r_TS [x, y, z]: " << T_ST_.inverse().translation().transpose());
  ROS_INFO_STREAM(
      "q_TS [x, y, z, w]: "
      << Eigen::Quaterniond(T_ST_.inverse().rotation()).coeffs().transpose());

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

void OdometryTransformer::broadcastCalibration() {
  geometry_msgs::TransformStamped tf = tf2::eigenToTransform(T_ST_);

  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = source_frame_;
  tf.child_frame_id = target_frame_;

  if (tf_static_br_.has_value())
    tf_static_br_->sendTransform(tf);
}

void OdometryTransformer::receiveOdometry(
    const nav_msgs::OdometryConstPtr &source_odometry) {
  ROS_INFO_ONCE("Received first odometry message.");

  // Get source pose in inertial/world coordinate frame.
  Eigen::Affine3d T_IS;
  Eigen::fromMsg(source_odometry->pose.pose, T_IS);

  // Compute pose of target frame.
  const Eigen::Affine3d T_IT = T_IS * T_ST_;

  // Compute linear twist of target frame.
  Eigen::Vector3d S_v_S, S_w_S;
  tf2::fromMsg(source_odometry->twist.twist.linear, S_v_S);
  tf2::fromMsg(source_odometry->twist.twist.angular, S_w_S);

  // Convert into inertial frame (not sure if this is necessary).
  const Eigen::Vector3d I_v_S = T_IS.rotation() * S_v_S;
  const Eigen::Vector3d I_w_S = T_IS.rotation() * S_w_S;
  const Eigen::Vector3d I_t_ST = T_IS.rotation() * T_ST_.translation();

  // Rigid body velocity.
  const Eigen::Vector3d I_v_T = I_v_S + I_w_S.cross(I_t_ST);
  const Eigen::Vector3d T_v_T = T_IT.rotation().inverse() * I_v_T;

  // The angular velocity is identical anywhere on the rigid body.
  const Eigen::Vector3d T_w_T = T_ST_.rotation().inverse() * S_w_S;

  // Convert to target odometry.
  nav_msgs::Odometry target_odometry;
  target_odometry.header = source_odometry->header;
  target_odometry.child_frame_id = target_frame_;
  target_odometry.pose.pose = Eigen::toMsg(T_IT);
  tf2::toMsg(T_v_T, target_odometry.twist.twist.linear);
  tf2::toMsg(T_w_T, target_odometry.twist.twist.angular);

  // TODO(rikba): Transform covariance.

  // Publish transformed odometry.
  odometry_pub_.publish(target_odometry);
}

} // namespace odometry_transformer
