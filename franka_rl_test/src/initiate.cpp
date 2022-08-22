#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/Float64.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
double w=0;

void jointCallback(const sensor_msgs::JointState::ConstPtr& state)
{
    double pos1=state->position[0];
    double pos2=state->position[1];
    w=pos1+pos2;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    //没有主题，就行形成一个主题，往对应topic里面发送东西
    //有主题，就用已经有的主题来发送东西
    ros::Publisher dn=node_handle.advertise<std_msgs::Bool>("/done_init", 10);
    ros::Subscriber sub=nh.subscribe("/joint_states",1000,jointCallback);
    spinner.start();

    std_msgs::Bool ifDone;

    // arm_group 的初始位置
    // grip_group 的初始位置
    std::vector<double> joint_group_positions={0,0,0,-1.3155,0,1.3155,0.785};
    std::vector<double> joint_grip_position={0.03,0.03};
    
    static const std::string PLANNING_GROUP="panda_arm_group";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    move_group.setJointValueTarget(joint_group_positions);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    
    static const std::string GRIP_GROUP="panda_gripper";
    moveit::planning_interface::MoveGroupInterface grip_group(GRIP_GROUP);
    grip_group.setJointValueTarget(joint_grip_position);

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        boost::shared_ptr<std_msgs::Bool const> sharedPtr=NULL;
        while(sharedPtr==NULL){
            sharedPtr=ros::topic::waitForMessage<std_msgs::Bool>("/make_init",nh);
        }

        std_msgs::Bool t=*sharedPtr;
        bool a=t.data;
        if(a){
            ROS_INFO("Moving to initial position...");
            usleep(2000000);
            move_group.setNamedTarget("initial");
            move_group.move();
            usleep(1000000);
            grip_group.move();
            ifDone.data=true;
            dn.publish(ifDone);
        }
        loop_rate.sleep();
    }
    grip_group.move();
}