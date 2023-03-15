#ifndef ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_
#define ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_

#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "odometry_transformer/OdometryTransformerConfig.h"

namespace odometry_transformer {
class OdometryTransformer {
public:
  OdometryTransformer(const ros::NodeHandle &nh,
                      const ros::NodeHandle &nh_private);
  void getRosParameters();
  void subscribeToRosTopics();
  void advertiseRosTopics();

private:
  void receiveOdometry(const nav_msgs::OdometryConstPtr &source_odometry);
  void receivePose(const geometry_msgs::PoseWithCovarianceStampedPtr &source_odometry);
  void broadcastCalibration();

  void initializeDynamicReconfigure();
  void reconfigureOdometryTransformer(
      odometry_transformer::OdometryTransformerConfig &config, uint32_t level);

  std::string source_frame_ = "";
  std::string target_frame_ = "";
  int queue_size_ = 1;
  bool tcp_no_delay_ = false;
  bool pose_transform_ = false;

  // Optionally publish TF and offer dynamic reconfigure if calibration is set
  // from ROS parameter server.
  std::optional<tf2_ros::StaticTransformBroadcaster> tf_static_br_;
  std::optional<dynamic_reconfigure::Server<
      odometry_transformer::OdometryTransformerConfig>>
      dyn_server_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // SE(3) Calibration from odometry source to target.
  Eigen::Affine3d T_ST_ = Eigen::Affine3d::Identity();

  ros::Subscriber odometry_sub_;
  ros::Publisher odometry_pub_;

  ros::Subscriber pose_sub_;
  ros::Publisher pose_pub_;
};
} // namespace odometry_transformer

#endif // ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_
