# odometry_transformer
A ROS package to transform odometry messages between two coordinate frames on a rigid body.

## Features
- Given known offset and rotation between odometry source and target frame, transform and publish odometry in target frame.
- Pass sensor calibration either by ROS parameter server or TF tree.
- Dynamically re-configure offset and orientation via [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure).
- Integration as ROS node, all parameters accessible as [roslaunch arguments](launch/odometry_transformer.launch).

## How to use.
The [odometry_transformer.launch](launch/odometry_transformer.launch) file shows how to startup the ROS node and pass all relevant parameters. The user can pass the calibration between the odometry source frame and odometry target frame either via parameter array or via [tf2_ros static_transform_publisher](http://wiki.ros.org/tf2_ros).

## Example
Convert the odometry (position, orientation, linear and angular velocity) measured by the [Realsense Tracking Camera T265](https://www.intelrealsense.com/tracking-camera-t265/) from the camera odometry frame to the robot base frame. For startup details see the [test_realsense_t265.launch](test/launch/test_realsense_t265.launch) file.
