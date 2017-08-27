#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "phidgets_ros/ActuatorStatesProcessed.h"
#include <math.h>


ros::Publisher joint_pub;

void sensorCallback(phidgets_ros::ActuatorStatesProcessed actuator_states)
{
	double arm_pos = actuator_states.arm;
	double bucket_pos = actuator_states.bucket;
	double sled_pos = actuator_states.sled;

        // distance from front to sled, chassis_length/2, sled_hoz/2 
	double sled_offset = -0.395 + (0.627/2) - (0.245/2);
	// Check for NaN, meaning we have not homed yet
	if (sled_pos != sled_pos) {
		sled_pos = 0;
	}
	else {
		sled_pos = (sled_pos/1000) + sled_offset; // convert mm to m and add offset, because back is 0
	}

	// distance from sled origin to front
	//double sled_offset = 0.627 / 2;

	// clip positions to boundary
	arm_pos = std::max(0.0, std::min(300.0, arm_pos));
	bucket_pos = std::max(0.0, std::min(150.0, bucket_pos));


	// These are not currently used
	double min_bucket = -1.693;
	double max_bucket = 0.384;

	// These values were obtained using a polyfit on data measured of the position and bucket angle - Matt
	// x = [32,40,48.5,60,70,80,90,100,110,120,130,140,150] # bucket linear actuator positions
	// y = [22,8,0,-12,-23,-31,-49,-50,-55,-65,-76,-86,-97] # angles. these were converted to radians
	double coeff3 = -8.572e-07;
	double coeff2 = 2.468e-04;
	double coeff1 = -3.814e-02;
	double coeff0 = 1.361;

	double bucket_angle = coeff3*pow(bucket_pos, 3) + coeff2*pow(bucket_pos, 2) + coeff1*bucket_pos + coeff0;

	// This is not the most accurate thing. Needs to be tweaked
	// 121.2121 is middle of joint (flat)
	double a = 350;
	double b = 410;
	double c = 370 + arm_pos;
	double arm_ang = -acos((a*a + b*b - c*c) / (2 * a * b)) + 1.3981;

	sensor_msgs::JointState joint_state;

	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(7);
	joint_state.position.resize(7);
	joint_state.name[0] = "sled_joint";
	joint_state.position[0] = sled_pos;
	joint_state.name[1] = "arm_joint";
	joint_state.position[1] = arm_ang;
	joint_state.name[2] = "bucket_joint";
	joint_state.position[2] = bucket_angle;
	joint_state.name[3] = "right_rear_wheel";
	joint_state.position[3] = 0;
	joint_state.name[4] = "right_front_wheel";
	joint_state.position[4] = 0;
	joint_state.name[5] = "left_rear_wheel";
	joint_state.position[5] = 0;
	joint_state.name[6] = "left_front_wheel";
	joint_state.position[6] = 0;

	joint_pub.publish(joint_state);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
		ros::NodeHandle nh;
		joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

		ros::Subscriber subPosition = nh.subscribe("/actuator_states/proc", 10, sensorCallback);

    sensor_msgs::JointState joint_state;

    while(ros::ok()){
		ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
