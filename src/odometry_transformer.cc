#include "odometry_transformer/odometry_transformer.h"

#include <functional>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>

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
    if (nh_private_.getParam("q_TS", q_TS) && q_TS.size() == 4) {
      // In Eigen the imaginary coefficient w is leading.
      T_TS.linear() = Eigen::Quaterniond(q_TS[3], q_TS[0], q_TS[1], q_TS[2])
          .normalized()
          .toRotationMatrix();
    } else {
      ROS_WARN_STREAM(log_param_error.c_str() << "q_TS");
    }
    if (nh_private_.getParam("T_r_TS", T_r_TS) && T_r_TS.size() == 3) {
      T_TS.translation() = Eigen::Vector3d(T_r_TS.data());
    } else {
      ROS_WARN_STREAM(log_param_error.c_str() << "T_r_TS");
    }
    T_ST_ = T_TS.inverse();

    // Broadcast TF if calibration is coming from parameter server.
    tf_static_br_.emplace();
    broadcastCalibration();

    // Enable dynamic reconfigure if calibration is coming from parameter
    // server.
    initializeDynamicReconfigure();
  }
  ROS_INFO_STREAM(
      "T_r_TS [x, y, z]: " << T_ST_.inverse().translation().transpose());
  ROS_INFO_STREAM(
      "q_TS [x, y, z, w]: "
          << Eigen::Quaterniond(T_ST_.inverse().rotation()).coeffs().transpose());

  nh_private_.getParam("queue_size", queue_size_);
  ROS_INFO("Odometry queue size: %d", queue_size_);

  nh_private_.getParam("tcp_no_delay", tcp_no_delay_);
  if (tcp_no_delay_) {
    ROS_INFO("TCP no delay activated.");
  } else {
    ROS_INFO("TCP no delay de-activated.");
  }

  // Pose transform as well
  nh_private_.getParam("pose_transform", pose_transform_);
  if (pose_transform_) {
    ROS_INFO("Pose transform activated.");
  } else {
    ROS_INFO("Pose transform de-activated.");
  }
}

void OdometryTransformer::subscribeToRosTopics() {
  odometry_sub_ = nh_.subscribe("source_odometry",
                                queue_size_,
                                &OdometryTransformer::receiveOdometry,
                                this,
                                tcp_no_delay_
                                ? ros::TransportHints().tcpNoDelay()
                                : ros::TransportHints());
  ROS_INFO("Subscribed to %s", odometry_sub_.getTopic().c_str());

  if(pose_transform_){
      pose_sub_ = nh_.subscribe("source_pose",
                                queue_size_,
                                &OdometryTransformer::receivePose,
                                this,
                                tcp_no_delay_
                                ? ros::TransportHints().tcpNoDelay()
                                : ros::TransportHints());
  ROS_INFO("Subscribed to %s", pose_sub_.getTopic().c_str());
  }
}

void OdometryTransformer::advertiseRosTopics() {
  odometry_pub_ =
      nh_.advertise<nav_msgs::Odometry>("target_odometry", queue_size_);
  ROS_INFO("Advertising %s", odometry_pub_.getTopic().c_str());

  pose_pub_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("target_pose", queue_size_);
  ROS_INFO("Advertising %s", pose_pub_.getTopic().c_str());
}

void OdometryTransformer::broadcastCalibration() {
  geometry_msgs::TransformStamped tf = tf2::eigenToTransform(T_ST_);

  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = source_frame_;
  tf.child_frame_id = target_frame_;

  if (tf_static_br_.has_value())
    tf_static_br_->sendTransform(tf);
}

void OdometryTransformer::initializeDynamicReconfigure() {
  // Manually convert current transform to dynamic reconfigure values.
  // IMPORTANT: Only if parameters are set before dynamic server is invoked
  // default values will be overwritten!
  Eigen::Vector3d rpy = T_ST_.inverse().rotation().eulerAngles(0, 1, 2);
  nh_private_.setParam("TS_roll", rpy.x());
  nh_private_.setParam("TS_pitch", rpy.y());
  nh_private_.setParam("TS_yaw", rpy.z());

  nh_private_.setParam("T_r_TS_x", T_ST_.inverse().translation().x());
  nh_private_.setParam("T_r_TS_y", T_ST_.inverse().translation().y());
  nh_private_.setParam("T_r_TS_z", T_ST_.inverse().translation().z());

  dynamic_reconfigure::Server<
      odometry_transformer::OdometryTransformerConfig>::CallbackType f;
  f = std::bind(&OdometryTransformer::reconfigureOdometryTransformer, this,
                std::placeholders::_1, std::placeholders::_2);
  dyn_server_.emplace(nh_private_);
  dyn_server_->setCallback(f);
}

void OdometryTransformer::reconfigureOdometryTransformer(
    odometry_transformer::OdometryTransformerConfig &config, uint32_t level) {
  ROS_DEBUG("Dynamically updating target frame to source frame transform.");
  Eigen::Affine3d T_TS = Eigen::Affine3d::Identity();

  T_TS.linear() =
      (Eigen::AngleAxisd(config.TS_roll, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(config.TS_pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(config.TS_yaw, Eigen::Vector3d::UnitZ()))
          .toRotationMatrix();

  T_TS.translation() =
      Eigen::Vector3d(config.T_r_TS_x, config.T_r_TS_y, config.T_r_TS_z);

  T_ST_ = T_TS.inverse();

  ROS_DEBUG_STREAM(
      "T_r_TS [x, y, z]: " << T_ST_.inverse().translation().transpose());
  ROS_DEBUG_STREAM(
      "q_TS [x, y, z, w]: "
          << Eigen::Quaterniond(T_ST_.inverse().rotation()).coeffs().transpose());

  // Update parameter array.
  const Eigen::Quaterniond q_TS = Eigen::Quaterniond(T_TS.linear());
  nh_private_.setParam(
      "q_TS", std::vector<double>({q_TS.x(), q_TS.y(), q_TS.z(), q_TS.w()}));
  nh_private_.setParam("T_r_TS",
                       std::vector<double>(T_TS.translation().data(),
                                           T_TS.translation().data() +
                                               T_TS.translation().size()));

  // Update TF.
  broadcastCalibration();
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

  // Rigid body linear velocity.
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

void OdometryTransformer::receivePose(
    const geometry_msgs::PoseWithCovarianceStampedPtr &source_pose) {
  ROS_INFO_ONCE("Received first pose message.");

  // Get source pose in inertial/world coordinate frame.
  Eigen::Affine3d T_IS;
  Eigen::fromMsg(source_pose->pose.pose, T_IS);

  // Compute pose of target frame.
  const Eigen::Affine3d T_IT = T_IS * T_ST_;

  // Convert to target odometry.
  geometry_msgs::PoseWithCovarianceStamped target_pose;
  target_pose.header = source_pose->header;
  target_pose.pose.pose = Eigen::toMsg(T_IT);

  // TODO(TimonMathis): Transform covariance.

  // Publish transformed odometry.
  pose_pub_.publish(target_pose);
}

} // namespace odometry_transformer
