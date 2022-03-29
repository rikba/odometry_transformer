#include <cmath>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
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
  tf.header.frame_id = "world";
  tf.child_frame_id = "camera";

  while (ros::ok()) {
    tf.header.stamp = ros::Time::now();

    tf.transform.translation.x =
        r * std::cos(2 * M_PI / T * tf.header.stamp.toSec());
    tf.transform.translation.y =
        r * std::sin(2 * M_PI / T * tf.header.stamp.toSec());
    tf.transform.translation.z =
        h * std::sin(2 * M_PI / T * tf.header.stamp.toSec());

    tf.transform.rotation.w = 1.0;

    tf_br_.sendTransform(tf);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
