#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>

#include <cmath>
#include <string>
#include <vector>
#include <iostream>

class TransformHelper{
public:
  TransformHelper();
  void publishConstantTransform();
  void tryToUpdate();
private:
  tf::TransformListener transform_listener;
  tf::StampedTransform odom_to_map;
  tf::TransformBroadcaster broadcaster;
};

TransformHelper::TransformHelper():
  transform_listener(),
  broadcaster()
{
  tf::Transform defaultTransform;
  defaultTransform.setOrigin(tf::Vector3(0,0,0));
  defaultTransform.setRotation(tf::Quaternion(1,1,1,0)); //maybe change to zeros?????
  odom_to_map.child_frame_id_ = "map";
  odom_to_map.frame_id_ = "odom";
  odom_to_map.stamp_ = ros::Time::now();
  odom_to_map.setData(defaultTransform);
}

void TransformHelper::publishConstantTransform(){
  try{
    odom_to_map.stamp_ = ros::Time::now();
    broadcaster.sendTransform(odom_to_map);
  }catch(tf::TransformException ex){
    ROS_ERROR("Error when publishing constant transform: %s", ex.what());
  }
}

void TransformHelper::tryToUpdate(){
  ros::Time latest = ros::Time(0);
  ros::Time now = ros::Time::now();

  tf::StampedTransform zed_to_map;
  try{
    transform_listener.waitForTransform("ZED_left_camera", "map", latest, ros::Duration(0.01));
    transform_listener.lookupTransform("ZED_left_camera", "map", latest, zed_to_map);
  }catch(tf::TransformException ex){
    ROS_ERROR("Could not lookup zed to map, canceling: %s", ex.what());
    return;
  }

  tf::StampedTransform camera_to_map;
  try{
    transform_listener.waitForTransform("camera_position", "map", latest, ros::Duration(0.01));
    transform_listener.lookupTransform("camera_position", "map", latest, camera_to_map);
  }catch(tf::TransformException ex){
    ROS_ERROR("Could not lookup camera_position to map, canceling: %s", ex.what());
    return;
  }

  tf::Vector3 errorOrigin = camera_to_map.getOrigin();
  tf::Vector3 thoughtOrigin = zed_to_map.getOrigin();
  errorOrigin += thoughtOrigin; // the -= operator is overloaded for vector3
  errorOrigin.setZ(0);

  tf::Vector3 newOrigin = odom_to_map.getOrigin();
  newOrigin += errorOrigin;
  tf::Transform updatedTransform;
  updatedTransform.setOrigin(newOrigin);

  tf::Quaternion zeroRotation;
  zeroRotation.setRPY(0,0,0);
  updatedTransform.setRotation(zeroRotation);
  odom_to_map = tf::StampedTransform(updatedTransform, now, "odom", "map");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "cole_helper");
  ros::NodeHandle nh;

  ros::Rate loop_rate(1);

  TransformHelper helper;

  while(ros::ok()){
    helper.tryToUpdate();
    helper.publishConstantTransform();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
