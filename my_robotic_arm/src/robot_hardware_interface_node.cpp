#include <my_robotic_arm/robot_hardware_interface.h>
#include "std_msgs/Float32.h"

float joint1_pos;
float joint2_pos;
float joint3_pos;

ROBOTHardwareInterface::ROBOTHardwareInterface(ros::NodeHandle& nh) : nh_(nh) {
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=4;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
	pub = nh_.advertise<rospy_tutorials::Floats>("/joints_to_aurdino",10);
	client = nh_.serviceClient<my_robotic_arm::Floats_array>("/read_joint_state");
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &ROBOTHardwareInterface::update, this);
}

ROBOTHardwareInterface::~ROBOTHardwareInterface() {
}

void ROBOTHardwareInterface::init() {
    
    num_joints_=3;
	joint_names_[0]="joint_1_s";	
	joint_names_[1]="joint_2_l";
	joint_names_[2]="joint_3_u";
	

    for (int i = 0; i < num_joints_; ++i) {

         // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create position joint interface
        hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
        
        position_joint_interface_.registerHandle(jointPositionHandle);
   
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void ROBOTHardwareInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void ROBOTHardwareInterface::read() {

    joint_position_[0] = joint1_pos;
    joint_position_[1] = joint2_pos;
    joint_position_[2] = joint3_pos;

    //ROS_INFO("Received Joint 1 Position [%f]", joint_position_[0]);
    //ROS_INFO("Received Joint 2 Position [%f]", joint_position_[1]);
    //ROS_INFO("Received Joint 3 Position [%f]", joint_position_[2]);



	//joint_read.request.req=1.0;
	//
	//if(client.call(joint_read))
	//{
	//    
	//	joint_position_[0]=angles::from_degrees(90-joint_read.response.res[0]);
	//	joint_position_[1]=angles::from_degrees(joint_read.response.res[1]-90);
	//	joint_position_[2]=angles::from_degrees(joint_read.response.res[2]-90);
	//	
	//	//ROS_INFO("Receiving  j1: %.2f, j2: %.2f, j3: %.2f",joint_read.response.res[0],joint_read.response.res[1], joint_read.response.res[2]);
	//	
	//}	
    //else
    //{
    //	joint_position_[0]=0;
    //    joint_position_[1]=0;
    //    joint_position_[2]=0;
    //    //ROS_INFO("Service not found ");
    //}
        

}

void ROBOTHardwareInterface::write(ros::Duration elapsed_time) {
    
	joints_pub.data.clear();
	joints_pub.data.push_back(90-(angles::to_degrees(joint_position_command_[0])));
	joints_pub.data.push_back(90 +(angles::to_degrees(joint_position_command_[1])));
	joints_pub.data.push_back(90 +(angles::to_degrees(joint_position_command_[2])));
	//ROS_INFO("Publishing j1: %.2f, j2: %.2f, j3: %.2f",joints_pub.data[0],joints_pub.data[1],joints_pub.data[2]);
	pub.publish(joints_pub);	
}

void joint_1_s_state_cb(const std_msgs::Float32::ConstPtr& msg)
{
    joint1_pos = msg->data;
    //ROS_INFO("Received [%f]", joint1_pos);
}

void joint_2_l_state_cb(const std_msgs::Float32::ConstPtr& msg)
{
    joint2_pos = msg->data;
}

void joint_3_u_state_cb(const std_msgs::Float32::ConstPtr& msg)
{
    joint3_pos = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_hardware_interface");
    ros::NodeHandle nh;
    ros::Subscriber joint1_state_sub = nh.subscribe ("/csim_simulation/robot_sim_env/joint_1_s/state", 10, joint_1_s_state_cb);
    ros::Subscriber joint2_state_sub = nh.subscribe ("/csim_simulation/robot_sim_env/joint_2_l/state", 10, joint_2_l_state_cb);
    ros::Subscriber joint3_state_sub = nh.subscribe ("/csim_simulation/robot_sim_env/joint_3_u/state", 10, joint_3_u_state_cb);

    //ros::AsyncSpinner spinner(2); 
    ros::MultiThreadedSpinner spinner(2);// 2 threads for controller service and for the Service client used to get the feedback from ardiuno
    //spinner.start();
    ROBOTHardwareInterface ROBOT(nh);
    spinner.spin();
    //ros::spin();
    return 0;
}
