#include <cmath>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_odometry_transformer");
  ros::NodeHandle nh_private("~");

  tf2_ros::TransformBroadcaster tf_br_;

  // Settings
  const double r = 3.0;
  const double h = 1.0;
  const double T = 30.0;
  ros::Rate loop_rate(10);

  geometry_msgs::TransformStamped tf;
  nav_msgs::Odometry odom;

  auto odom_pub =
      nh_private.advertise<nav_msgs::Odometry>("camera_odometry", 1);

  while (ros::ok()) {
    const auto t = ros::Time::now();
    const auto s = std::sin(2 * M_PI / T * tf.header.stamp.toSec());
    const auto c = std::cos(2 * M_PI / T * tf.header.stamp.toSec());
    auto T_WC = Eigen::Affine3d::Identity();

    // Position.
    T_WC *= Eigen::Translation3d(r * c, r * s, h * s);

    // Velocity.

    // Broadcast.
    // tf2
    tf = tf2::eigenToTransform(T_WC);
    tf.header.stamp = t;
    tf.header.frame_id = "world";
    tf.child_frame_id = "camera";
    tf_br_.sendTransform(tf);

    // odometry
    odom.pose.pose = tf2::toMsg(T_WC);
    odom.header = tf.header;
    odom_pub.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
