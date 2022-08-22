
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/Float64.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <math.h>
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <std_msgs/Bool.h>

bool d = false;

void Callback(const std_msgs::Bool::ConstPtr& data) {
  d = data->data;
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"make_check");
    ros::NodeHandle node_handle;
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Publisher pub=node_handle.advertise<std_msgs::Bool>("/done_checking",10);
    
    static const std::string PLANNING_GROUP="panda_arm_group";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    const robot_state::JointModelGroup *joint_model_group=move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped target_pose;

    current_pose=move_group.getCurrentPose();

    ros::Rate loop_rate(100);
    int k=1;

    while(ros::ok()){
      boost::shared_ptr<std_msgs::Bool const> sharePtr=NULL;
      
      // while(sharePtr==NULL){
      //     sharePtr=ros::topic::waitForMessage<std_msgs::Bool>("/make_check",nh);
      // }

      // std_msgs::Bool t=*sharePtr;
      std_msgs::Bool t;
      t.data=true;
      bool a=t.data;
      if(a){
        current_pose=move_group.getCurrentPose();
        target_pose=current_pose;
        target_pose.pose.position.z=target_pose.pose.position.z-0.17; //往z轴走17cm
        move_group.setApproximateJointValueTarget(target_pose);
        move_group.move();
        std_msgs::Bool msg;
        msg.data=true;
        std::cout<<"got in"<<'\n';
        usleep(1000000);
        pub.publish(msg);
        k++;
      }
      loop_rate.sleep();
    }

}