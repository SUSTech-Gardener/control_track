//#include "ros/init.h"
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

int main(int argc,char **argv)
{
ros::init(argc,argv,"soem_publisher")   ;
ros::NodeHandle nh;
std_msgs::Int32MultiArray velocity_msg ;
velocity_msg.data.resize(2);
velocity_msg.data[0]=10000;
velocity_msg.data[1]=10000;
ros::Publisher pub = nh.advertise<std_msgs::Int32MultiArray>("vel",1000);
ros::Rate loop_rate(1.0);
while (ros::ok)
{
    pub.publish(velocity_msg);
    loop_rate.sleep();
}
return 0;


}