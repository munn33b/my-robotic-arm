#include "dynamixel_hardware/dynamixel_hardware.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <custom_message/msg/floats.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


namespace dynamixel_hardware
{

hardware_interface::CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    num_joints_ = info.joints.size();
    position_commands_.resize(num_joints_, 0.0);
    position_states_.resize(num_joints_, 0.0);

    // ROS2 Node initialization
    node_ = rclcpp::Node::make_shared("dynamixel_hardware");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });

    //Subscriber to Joint States
    joint1_state_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
      "/csim_simulation/robot_sim_env/joint_1_s/state", 10,
      std::bind(&DynamixelHardware::joint1_state_callback, this, std::placeholders::_1));
    joint2_state_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/csim_simulation/robot_sim_env/joint_2_l/state", 10,
        std::bind(&DynamixelHardware::joint2_state_callback, this, std::placeholders::_1));
    joint3_state_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/csim_simulation/robot_sim_env/joint_3_u/state", 10,
        std::bind(&DynamixelHardware::joint3_state_callback, this, std::placeholders::_1));

    // Subscriber for encoder_angle
    target_angle_publisher_ = node_->create_publisher<custom_message::msg::Floats>(
        "/joints_to_aurdino", 10);


    RCLCPP_INFO(node_->get_logger(), "Hardware interface initialized with ROS2 Subscriber for encoder_angle.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// Callback function to update position_states_
void DynamixelHardware::joint1_state_callback(const std_msgs::msg::Float32::SharedPtr msg)
{

    position_states_[0] = msg->data;  // Update joint positions

}
void DynamixelHardware::joint2_state_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    position_states_[1] = msg->data;  // Update joint positions
}
void DynamixelHardware::joint3_state_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    position_states_[2] = msg->data;  // Update joint positions
}


std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < num_joints_; i++)
    {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, "position", &position_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < num_joints_; i++)
    {
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, "position", &position_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type DynamixelHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    for (size_t i = 0; i < num_joints_; i++)
    {
        position_states_[i] = position_states_[i];
        RCLCPP_DEBUG(node_->get_logger(), "Joint %zu state: %f", i, position_states_[i]);
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    // Create a single message to contain all joint commands
    auto msg = custom_message::msg::Floats();
    msg.data.resize(position_commands_.size());

    // Convert radians -> degrees and apply offsets
    for (size_t i = 0; i < position_commands_.size(); i++)
    {
        float deg = static_cast<float>(angles::to_degrees(position_commands_[i]));

        // Apply offset rules (example as in your snippet)
        if (i == 0)
            msg.data[i] = 90.0f - deg;
        else if (i == 1 || i == 2)
            msg.data[i] = 90.0f + deg;
        else
            msg.data[i] = deg;  // default: just degrees
    }

    // Publish the complete command array once
    target_angle_publisher_->publish(msg);

    // Optionally, log the commands for debugging purposes
    for (size_t i = 0; i < num_joints_; i++)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Joint %zu command (deg): %f", i, msg.data[i]);
    }

    return hardware_interface::return_type::OK;
}



void DynamixelHardware::send_command_to_motor(size_t joint_index, double position)
{
    auto msg = custom_message::msg::Floats();
    msg.data.resize(num_joints_);  // Resize to match the number of joints
    msg.data[joint_index] = position;  // Assign the command to the correct index
    target_angle_publisher_->publish(msg);
}



} // namespace dynamixel_hardware

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)

