#include <math.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <iostream>

double w=0;

void jointCallback(const sensor_msgs::JointState::ConstPtr& state){
    double pos1=state->position[0];
    double pos2=state->position[1];
    w=pos1+pos2;
}

int main(int argc,char **argv)
{
   ros::init(argc,argv,"grip");
   ros::NodeHandle nh;
   ros::Subscriber sub=nh.subscribe("/joint_states",1000,jointCallback);

   ros::Rate loop_rate(10);
   while(ros::ok())
   {
       std::cout<<"======================"<<std::endl;
       std::cout<<w<<'\n';
       loop_rate.sleep();
       ros::spinOnce();
   }

   return 0;
}
