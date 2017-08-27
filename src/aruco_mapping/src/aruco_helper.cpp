// @file aruco_helper.cpp
// @brief Backtraces transform based on camera input to snap odom to world map
// @author Matthew Wilson (matwilso)
//
// This gets pretty deep into the transforms and implementation details that might
// change, but oh well.  This also may not work well and has not been tested
// extensively

// WHAT THIS CODE DOES:
//
// When we are navigating, we always want to be navigating with respect to the map.
// We do not start with this transform and when we are driving, our odometry is
// not perfect, so we drift.  We need the transform from map (global frame) to
// odom (frame that represents where we started).
//
// Using the measured transform of the camera_position from the map frame,
// acquired by the aruco_mapping, and the transform from the ZED_left_camera
// from the odom frame, since the camera_position and the ZED_left_camera
// are the exact same point, we can calculate the transform from map to odom.
//
// Booyah.  How do we do that?
//
// This node listens to two transforms: the transform from the camera position to
// the map (published by aruco_mapping node) and that from the odom frame to the
// ZED_left_camera frame (published by odometry node and robot publisher + URDF).
// Here is some accompanying ASCII art.
//
//   map (world 0,0)        odom (where the robot started)
//       |                               |
//       |                               |
//       v                               v
// camera_position      ==        ZED_left_camera
//
//
// (map -> camera_position) - (odom -> ZED_left_camera) = map -> odom

// There are a few other details related to the tf libray, but that is good for
// now


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "aruco_helper");

  ros::NodeHandle node;
  tf::TransformListener listener;
  static tf::TransformBroadcaster br;  // have to declare it static for some reasons
  tf::Transform odom_map_tf;

  // flag to indicate we have received at least 1 transform,
  // so we can start republishing that information
  bool transform_is_set = false;
  // this is a terrible code, magic number 500 that is chosen for no reason
  // I needed a number to initialze the value to that it would never
  // naturally be so that I would know if it wasn't set yet.
  // plz FIXME
  double last_yaw = 500.;

  double qx, qy, qz, qw;
  ros::param::get("~qx", qx);
  ros::param::get("~qy", qy);
  ros::param::get("~qz", qz);
  ros::param::get("~qw", qw);

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform odom_tree_transform;
    tf::StampedTransform map_tree_transform;

    if (listener.canTransform("camera_position", "map", ros::Time::now()-ros::Duration(0.1)) &&
     listener.canTransform("ZED_left_camera", "odom", ros::Time::now()-ros::Duration(0.1))) {

       transform_is_set = true;
       listener.lookupTransform("ZED_left_camera", "odom", ros::Time(0), odom_tree_transform);
    //    listener.lookupTransform("odom", "ZED_left_camera", ros::Time(0), odom_tree_transform);
       listener.lookupTransform("camera_position", "map", ros::Time(0), map_tree_transform);
    //    listener.lookupTransform("map", "camera_position", ros::Time(0), map_tree_transform);

    //    Calculated the different between the map and odom frames
    //    (map -> camera_position) - (odom -> ZED_left_camera) = map -> odom
        odom_map_tf = map_tree_transform.inverseTimes(odom_tree_transform);
        // odom_map_tf = odom_tree_transform.inverseTimes(map_tree_transform);


        // Create a quaternion from the parameters passed in that will be used
        // to rotate the transform
        // The launch file has tranform of 0 degrees.  This should probably
        // be taken out when the system can be tested again
        // TODO ^
        tf::Quaternion rot = tf::Quaternion(qx, qy, qz, qw);

        // Rotate the odom map difference and normalize the quaternion
        odom_map_tf.setRotation((odom_map_tf.getRotation()*rot).normalize());
        tf::Quaternion quat = odom_map_tf.getRotation();

        // Conver the quaterion to RPY so we can check the limits and remove
        // everything besides the rotation about Z axis.

        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // This check was necessary because the rotation causes the tranform
        // to have weird discontinuites where the robot position completely flips
        if (last_yaw == 500.)  {
            last_yaw = yaw;
        }
        if (abs(yaw - last_yaw) > 0.785) {
            ROS_WARN("Current Yaw: %.2f, Last: %.2f", yaw, last_yaw);
           rate.sleep();
           continue;
        }

        odom_map_tf.setRotation(tf::Quaternion(0.0, 0.0, yaw));
        tf::Vector3 translation = odom_map_tf.getOrigin();
        odom_map_tf.setOrigin(tf::Vector3(translation.getX(), translation.getY(), 0.0));
        br.sendTransform(tf::StampedTransform(odom_map_tf, ros::Time::now(), "map", "odom"));
        last_yaw = yaw;
    }
    else {
        // If we have received a valid transform, keep publishing that until you see a new one
        // This allows us to retain state when we are facing away.
      if (transform_is_set) {
        br.sendTransform(tf::StampedTransform(odom_map_tf, ros::Time::now(), "map", "odom"));
      }
      // We haven't received anything yet, publish 0,0.
      else {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
      }
    }
    rate.sleep();
  }
  return 1;
};
