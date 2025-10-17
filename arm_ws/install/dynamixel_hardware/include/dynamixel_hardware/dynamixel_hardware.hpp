#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP

#include "hardware_interface/system_interface.hpp"
#include "custom_message/msg/floats.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <angles/angles.h>


namespace dynamixel_hardware
{
class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    size_t num_joints_;  // Number of joints
    std::vector<double> position_commands_; // Target angles (commands)
    std::vector<double> position_states_;   // Encoder angles (states)

    // ROS 2 Publisher and Subscriber
    rclcpp::Publisher<custom_message::msg::Floats>::SharedPtr target_angle_publisher_; // Publishes target_angle
    // Declare the subscriber for joint_states
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint1_state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint2_state_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint3_state_subscriber_;


    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;

    // ROS 2 Node
    rclcpp::Node::SharedPtr node_;

    // Send command to motor
    void send_command_to_motor(size_t joint_index, double position);
    void joint1_state_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void joint2_state_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void joint3_state_callback(const std_msgs::msg::Float32::SharedPtr msg);
};
} // namespace dynamixel_hardware

#endif // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP

