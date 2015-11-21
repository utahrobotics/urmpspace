#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <vector>


void drive_Callback(const std_msgs::Float32MultiArray::ConstPtr& vals){
	float arr[5] = {1.0, 2.3, -3.7, -0.5, 0.67};
	//for(int i = 0; i < 5; i++){
	//	arr[i] = vals[i];
	//}
	int i = 0;
	for(std::vector<float>::const_iterator it = vals->data.begin(); it != vals->data.end(); ++it)
	{
		arr[i] = *it;
		i++;
	}

	ROS_INFO("Left motor: %f", arr[1]);
	ROS_INFO("Right motor: %f", arr[1]);
	ROS_INFO("Arm motor: %f", arr[2]);
	ROS_INFO("Tilt motor: %f", arr[3]);
	ROS_INFO("Slide motor: %f", arr[4]);
	return;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "drive");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("motor_vals", 1000, drive_Callback);
	ros::spin();
	return 0;
}