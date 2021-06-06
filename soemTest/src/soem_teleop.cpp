#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

// create the TeleopTurtle class and define the joyCallback function that will take a joy msg
class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  ros::NodeHandle nh_;

  int linear_, angular_;   // used to define which axes of the joystick will control our turtle
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

int main(int argc, char** argv)
{
  // initialize our ROS node, create a teleop_turtle, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "soem_teleop");
  TeleopTurtle teleop_turtle;

  ros::spin();
}



TeleopTurtle::TeleopTurtle(): linear_(1), angular_(2)
{
  //  initialize some parameters
  nh_.param("axis_linear", linear_, linear_);  
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  // create a publisher that will advertise on the command_velocity topic of the turtle
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("track_cmd_vel", 1);

  // subscribe to the joystick topic for the input to drive the turtle
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);
}


void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;

  // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the turtle
  twist.angular.z = a_scale_*joy->axes[3]+2*a_scale_*joy->axes[6];
  twist.linear.x = l_scale_*joy->axes[1]+2*l_scale_*joy->axes[7];
  twist.linear.y = l_scale_*joy->axes[0];
  
  vel_pub_.publish(twist); 

}
