#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <time.h>

int main(int argc, char **argv)
{
    
	std::srand(time(0));
	ros::init(argc, argv, "talk");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("motor_vals", 100);

	while (ros::ok())
	{
		std_msgs::Float32MultiArray array;
		//Clear array
		array.data.clear();
		//for loop, pushing data in the size of the array
		for (int i = 0; i < 5; i++)
		{
			//assign array a random number between 0 and 255.
			array.data.push_back((rand()%100)*0.01);
		}
		//Publish array
		pub.publish(array);
		//Let the world know
		ROS_INFO("I published data!");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}

}