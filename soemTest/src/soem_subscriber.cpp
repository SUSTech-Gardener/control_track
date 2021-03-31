
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
ros::Subscriber velSub;

std_msgs::Int32MultiArray velMsg;

int32_t vel1,vel2;

void velCallback(std_msgs::Int32MultiArray velMsg)
{
    vel1 = velMsg.data[0];
    vel2 = velMsg.data[1];
    ROS_INFO("%i,%i",vel1,vel2);
}
int main(int argc,char **argv)
{   
    int vel1 = 0 ;
    int vel2 = 0 ; 
    ros::init(argc,argv,"soem_subscriber");           //疑惑的点   这里的node应该是真实的node吧？
    ros::NodeHandle n;
    velSub = n.subscribe("vel",100,velCallback);
    ros::spin() ;
    printf("start\n");
    return (0);
    }
//////////////////////////////////////////////////////////////////////////////////////////////
 
//////////////////////////////////////////////////////////////////////////////////////////////