#include <ros/ros.h>
#include <serial/serial.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>

#include <signal.h>
#include <cmath>
#include <string>
#include <vector>
#include <iostream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>


//serial::Serial teensy;
class TeensySerial{
public:
  TeensySerial();
  serial::Serial teensy;
  void printFlush();
  void sendConstantTransform();
  float angle_offset;
private:
  int baud;
  std::string port;
  tf::TransformListener transform_listener;
  tf::StampedTransform map_to_odom;
  tf::TransformBroadcaster broadcaster;
  int last_message_from;
  float last_x;
  float last_y;
  float last_angle;
  void generateTransform(float x, float y, int sensor_number, float angle);
  float calculateAngle(float x1, float y1, float x2, float y2);
  static const float y_offset = -0.016;
  static const float x_offset = 2.494;
};

TeensySerial::TeensySerial():
  teensy(),
  transform_listener(),
  broadcaster()
{
  baud = 9600;
  port = std::string("/dev/ttyACM0");
  //teensy = new serial::Serial(port, baud);
  teensy.setPort(port);
  teensy.setBaudrate(baud);
  try{
    teensy.open();
  }catch(serial::IOException ex){
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Could Not make a connection to the teensy");
  }
  tf::Transform defaultTransform;
  defaultTransform.setOrigin(tf::Vector3(0,0,0));
  defaultTransform.setRotation(tf::Quaternion(1,1,1,0));
  map_to_odom.child_frame_id_ = "map";
  map_to_odom.frame_id_ = "odom";
  map_to_odom.stamp_ = ros::Time::now();
  map_to_odom.setData(defaultTransform);

  last_message_from = -1;
  last_x = 0;
  last_y = 0;
  last_angle = 0;
  angle_offset = 0;
}

void TeensySerial::sendConstantTransform(){
  try{
    //collector_bin_to_map.frame_id_ =  "collection_bin_map";
    map_to_odom.stamp_ = ros::Time::now();
    //ROS_INFO("Sending Transform: x: %f, y %f", map_to_odom.getOrigin.x(), map_to_odom.getOrigin.y());
    broadcaster.sendTransform(map_to_odom);
  }catch(tf::TransformException ex){
    ROS_ERROR("%s", ex.what());
  }

}

void TeensySerial::generateTransform(float x, float y, int sensor_number, float angle){
  //static tf::TransformBroadcaster broadcaster;
  ros::Time now = ros::Time::now();
  ros::Time latest = ros::Time(0);
  tf::StampedTransform baselink_odom;
  try{
    transform_listener.waitForTransform("base_link", "odom", latest, ros::Duration(0.01));
    transform_listener.lookupTransform("base_link", "odom", latest, baselink_odom);
  }catch(tf::TransformException ex){
    ROS_ERROR("Could not get rotation transform from base_link to odom");
    ROS_ERROR("%s", ex.what());
  }
  tf::Quaternion base_rotation = baselink_odom.getRotation().normalize();
  tf::Quaternion offset;
  offset.setRPY(0,0,angle_offset);
  tf::Quaternion new_rotation = base_rotation + offset; // may need to subtract here

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x + x_offset, y + y_offset, 0));
  tf::Quaternion rotation;
  //rotation.setRPY(0, 0, angle_offset); //not used, I think...
  transform.setRotation(new_rotation);

  ros::Time timeOfTransform = ros::Time::now();
  switch(sensor_number){
    case 0:
      broadcaster.sendTransform(tf::StampedTransform(transform, timeOfTransform, "beacon_back_left", "collection_bin_robot_0"));
      break;
    case 1:
      broadcaster.sendTransform(tf::StampedTransform(transform, timeOfTransform, "beacon_back_right", "collection_bin_robot_1"));
      break;
    case 2:
      broadcaster.sendTransform(tf::StampedTransform(transform, timeOfTransform, "beacon_back_left", "collection_bin_robot_2"));
      break;
  }

  try{
    transform_listener.waitForTransform("beacon_back_left", "collection_bin_robot_0", timeOfTransform, ros::Duration(0.1));
  }catch(tf::TransformException ex){
    ROS_ERROR("could not wait for the transform to be set");
  }

  tf::StampedTransform baselink_collectorbin;

  switch (sensor_number) {
    case 0:
      try{
        transform_listener.waitForTransform("base_link", "collection_bin_robot_0", latest, ros::Duration(0.01));
        transform_listener.lookupTransform("base_link", "collection_bin_robot_0", latest, baselink_collectorbin);
      }catch(tf::TransformException ex){
        ROS_ERROR("Could not get transform between base_link and collection_bin_robot_0");
        ROS_ERROR("%s", ex.what());
      }
     break;
    case 1:
      try{
        transform_listener.waitForTransform("base_link", "collection_bin_robot_1", latest, ros::Duration(0.01));
        transform_listener.lookupTransform("base_link", "collection_bin_robot_1", latest, baselink_collectorbin);
      }catch(tf::TransformException ex){
        ROS_ERROR("Could not get transform between base_link and collection_bin_robot_1");
        ROS_ERROR("%s", ex.what());
      }
      break;
    case 2:
      try{
        transform_listener.waitForTransform("base_link", "collection_bin_robot_2", latest, ros::Duration(0.01));
        transform_listener.lookupTransform("base_link", "collection_bin_robot_2", latest, baselink_collectorbin);
      }catch(tf::TransformException ex){
        ROS_ERROR("Could not get transform between base_link and collection_bin_robot_2 ");
        ROS_ERROR("%s", ex.what());
      }
      break;
  }

  // try{
  //   transform_listener.waitForTransform("base_link", "collection_bin_robot_0", latest, ros::Duration(0.01));
  //   transform_listener.lookupTransform("base_link", "collection_bin_robot_0", latest, baselink_collectorbin);
  // }catch(tf::TransformException ex){
  //   ROS_ERROR("Could not get transform between base_link and collection_bin_robot_0");
  //   ROS_ERROR("%s", ex.what());
  // }

  tf::StampedTransform baselink_map;
  try{
    transform_listener.waitForTransform("base_link", "map", latest, ros::Duration(0.01));
    transform_listener.lookupTransform("base_link", "map", latest, baselink_map);
  }catch(tf::TransformException ex){
    ROS_ERROR("Could not get tranform between base_link and map");
    ROS_ERROR("%s", ex.what());
  }

  tf::Vector3 knownOrigin = baselink_collectorbin.getOrigin();
  tf::Vector3 thoughtOrigin = baselink_map.getOrigin();
  knownOrigin -= thoughtOrigin; // the -= operator is overloaded for vector3
  // float dx = knownOrigin.x() - thoughtOrigin.x();
  // float dy = knownOrigin.y() - thoughtOrigin.y();
  // float dz = knownOrigin.z() - thoughtOrigin.z();
  knownOrigin.setZ(0);
  //knownOrigin.setX(-knownOrigin.x());
  //knownOrigin.setY(-knownOrigin.y());
  ROS_INFO("Error is X: %f, Y: %f", knownOrigin.x(), knownOrigin.y());
  // ROS_INFO("Known: x: %f, y: %f", knownOrigin.x(), knownOrigin.y());
  // ROS_INFO("Though: x: %f, y: %f", thoughtOrigin.x(), thoughtOrigin.y());
  tf::Transform updatedOffset;
  tf::Quaternion zeroRotation;
  zeroRotation.setRPY(0,0,0);
  // updatedOffset.setOrigin(knownOrigin);
  tf::Vector3 newOrigin = map_to_odom.getOrigin();
  newOrigin += knownOrigin;
  updatedOffset.setOrigin(newOrigin);
  tf::Quaternion negative_new_rotation = -new_rotation;
  updatedOffset.setRotation(negative_new_rotation);
  map_to_odom = tf::StampedTransform(updatedOffset, ros::Time::now(), "odom", "map");

}

float TeensySerial::calculateAngle(float x1, float y1, float x2, float y2){
  float top = abs(y2 - y1);
  float bottom = abs(x2 - x1);
  float quot = top/bottom;
  return atan(quot); //arctangent from math library
}

void TeensySerial::printFlush(){
  std::string line;
  std::vector<std::string> split_line;
  std::string x;
  std::string y;
  std::string z;
  if(teensy.available()){
    line = teensy.readline();
    if(line[0] == 'P'){
      //ROS_INFO("%s", line.c_str());
      boost::algorithm::split(split_line, line, boost::algorithm::is_any_of(" "));
      x = split_line[3];
      y = split_line[1];
      std::string sensor_number = split_line[5];
      float fx = std::atof(x.c_str());
      float fy = std::atof(y.c_str());
      float fsensor_number = std::atof(sensor_number.c_str());
      float tempx = fx + x_offset;
      float tempy = fy + y_offset;
      ROS_INFO("X Position: %f m \t Y Position: %f m \t Sensor: %f", tempx, tempy, fsensor_number);

      if(last_message_from == -1 ){
        last_message_from = fsensor_number;
      }
      if(last_message_from != fsensor_number){
        if(last_message_from == 0){
          last_angle = calculateAngle(fx, fy, last_x, last_y);
          generateTransform(fx, fy, fsensor_number, last_angle);
        }else{
          last_angle = calculateAngle(last_x, last_y, fx, fy);
          generateTransform(fx, fy, fsensor_number, last_angle);
        }
      }else{
        generateTransform(fx, fy, fsensor_number, last_angle);
      }
    }else{
      ROS_INFO("I got something other than a position message: %s", line.c_str());
    }
    teensy.flushInput();
  }
}

TeensySerial* teensy_1;

void sigHandle(int sig){
  teensy_1->teensy.close();
  delete teensy_1;
  ROS_INFO("Closing Node");
  ros::shutdown();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "beacon_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  signal(SIGINT, sigHandle);
  teensy_1 = new TeensySerial();
  if(teensy_1->teensy.isOpen()){
    ROS_INFO("Connection with teensy Good");
  }else{
    ROS_INFO("Connection with teensy Bad");
    ros::shutdown();
    return -1;
  }

  ros::Rate loop_rate(1);

  while(ros::ok()){
    nh.getParam("initial_angle", teensy_1->angle_offset);
    teensy_1->sendConstantTransform();
    teensy_1->printFlush();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
