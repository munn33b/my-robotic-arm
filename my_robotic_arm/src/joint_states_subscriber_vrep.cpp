#include "ros/ros.h"
#include "std_msgs/Float32.h"



void joint_1_s_state_cb(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Received [%f]", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vrep_joint_state_subscriber");
    ros::NodeHandle nodeobj;
    ros::Subscriber joint_state_sub = nodeobj.subscribe("/csim_simulation/robot_sim_env/joint_1_s/state", 10, joint_1_s_state_cb);
    ros::spin();
    return 0;


}