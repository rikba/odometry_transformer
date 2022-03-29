#ifndef ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_
#define ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_

#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace odometry_transformer {
class OdometryTransformer {
public:
  OdometryTransformer(const ros::NodeHandle &nh,
                      const ros::NodeHandle &nh_private);
  void getRosParameters();
  void subscribeToRosTopics();

private:
  void receiveOdometry(const nav_msgs::OdometryConstPtr &msg);

  std::string source_frame_ = "";
  std::string target_frame_ = "";
  int queue_size_ = 1;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber odometry_sub_;
  ros::Publisher odometry_pub_;
};
} // namespace odometry_transformer

#endif // ODOMETRY_TRANSFORMER_ODOMETRY_TRANSFORMER_H_
