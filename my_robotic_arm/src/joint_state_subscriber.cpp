#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

void joint_states_cb(const sensor_msgs::JointState::ConstPtr& js_msg)
{

    ROS_INFO("Received [%double]", js_msg->position);

}

int main (int argc, char **argv)
{

    ros::init(argc, argv, "joint_state_subscriber");
    ros::NodeHandle nodehandle;
    ros::Subscriber joint_state_subscriber = nodehandle.subscribe ("/joint_states", 10, joint_states_cb); 
    return 0;

}
