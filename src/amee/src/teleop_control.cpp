#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

class TeleopControl{
public:
  TeleopControl();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  // Axes on the controller
  int left_vertical_axis_, right_vertical_axis_;
  int sled_forward_Y_, sled_backward_A_;
  int left_shoulder_, right_shoulder_;
  int left_trigger_, right_trigger_;
  int back_button_, start_button_;
  bool back_btn_last_val;
  bool start_btn_last_val;
  ros::Time callback_last_time; 

  // Scaling Factors to put the twist message in the correct range expected by
  // the Roboclaw controllers. Set to one for now.
  double linear_scale_, angular_scale_;

  ros::Publisher cmd_velocity_publisher_;
  ros::Publisher sled_velocity_publisher_;
  ros::Publisher arm_velocity_publisher_;
  ros::Publisher bucket_velocity_publisher_;
  ros::Publisher back_button_publisher_;
  ros::Publisher start_button_publisher_;

  ros::Subscriber joy_subscriber_;
};

TeleopControl::TeleopControl():
  left_vertical_axis_(1),
  right_vertical_axis_(4),
  sled_forward_Y_(3),
  sled_backward_A_(0),
  left_shoulder_(4),
  right_shoulder_(5),
  left_trigger_(2),
  right_trigger_(5),
  back_button_(6),
  start_button_(7)
{
  back_btn_last_val = 0;
  start_btn_last_val = 0;
  callback_last_time = ros::Time::now();

  nh_.param("left_vertical_axis", left_vertical_axis_, left_vertical_axis_); //get the axis for the linear component, or use our default of 2
  nh_.param("right_vertical_axis", right_vertical_axis_, right_vertical_axis_);
  nh_.param("sled_forward_button", sled_forward_Y_, sled_forward_Y_);
  nh_.param("sled_backward_button", sled_backward_A_, sled_backward_A_);
  nh_.param("bucket_forward_button", left_shoulder_, left_shoulder_);
  nh_.param("bucket_backward_button", right_shoulder_, right_shoulder_);
  nh_.param("arm_forward_axis", left_trigger_, left_trigger_);
  nh_.param("arm_backward_axis", right_trigger_, right_trigger_);

  nh_.param("scale_angular", angular_scale_, angular_scale_);
  nh_.param("scale_linear", linear_scale_, linear_scale_);

  cmd_velocity_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1); //publish to cmd_vel with a queue size of 1
  sled_velocity_publisher_ = nh_.advertise<std_msgs::Float32>("/sled/vel", 1);
  arm_velocity_publisher_ = nh_.advertise<std_msgs::Float32>("/arm/vel", 1);
  bucket_velocity_publisher_ = nh_.advertise<std_msgs::Float32>("/bucket/vel", 1);
  start_button_publisher_ = nh_.advertise<std_msgs::Empty>("/click/start_button", 1); //publish a single message every button press
  back_button_publisher_ = nh_.advertise<std_msgs::Empty>("/click/back_button", 1);
  joy_subscriber_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopControl::joyCallback, this);
}

void TeleopControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  ros::Time now_time = ros::Time::now();
  bool back_btn_curr = joy->buttons[back_button_];
  bool start_btn_curr = joy->buttons[start_button_];
  if (back_btn_last_val == 0 and back_btn_curr == 1) {
      back_button_publisher_.publish(std_msgs::Empty());
      back_btn_last_val = 1;
  }
  else {
      back_btn_last_val = 0;
  }
  if (start_btn_last_val == 0 and start_btn_curr == 1) {
      start_button_publisher_.publish(std_msgs::Empty());
      start_btn_last_val = 1;
  }
  else {
      start_btn_last_val = 0;
  }

  geometry_msgs::Twist twist;
  std_msgs::Float32 sled;
  std_msgs::Float32 arm;
  std_msgs::Float32 bucket;

  twist.linear.x = linear_scale_ * (joy->axes[left_vertical_axis_] + joy->axes[right_vertical_axis_]); // has max value of 2
  twist.angular.z = angular_scale_ * (joy->axes[left_vertical_axis_] - joy->axes[right_vertical_axis_]); // has max value of 2
  sled.data = (joy->buttons[sled_backward_A_] - joy->buttons[sled_forward_Y_]);
  arm.data = ((joy->axes[right_trigger_]+1)/2) - ((joy->axes[left_trigger_]+1)/2);
  bucket.data = joy->buttons[left_shoulder_] - joy->buttons[right_shoulder_];

  cmd_velocity_publisher_.publish(twist);
  sled_velocity_publisher_.publish(sled);
  arm_velocity_publisher_.publish(arm);
  bucket_velocity_publisher_.publish(bucket);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "teleop_control");
  TeleopControl teleop_control;

  ros::spin();
}
