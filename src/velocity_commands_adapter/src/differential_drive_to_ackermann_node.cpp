#include <cstdio>
#include <velocity_commands_adapter/differential_drive_to_ackermann_node.hpp>

namespace velocity_commands_adapter {

VelocityCommandsAdapterNode::VelocityCommandsAdapterNode() : Node("velocity_commands_adapter_node") {
  initialize();
}

void VelocityCommandsAdapterNode::initialize() {
  // declare and read ros parameters
  this->declare_parameter("cmd_vel_differential_drive_topic", "/cmd_vel_differential_drive");
  this->declare_parameter("cmd_vel_ackermann_topic", "/cmd_vel_ackermann");
  this->declare_parameter("wheel_base", 0.5);
  diff_cmd_vel_topic_ = this->get_parameter("cmd_vel_differential_drive_topic").as_string();
  ackermann_cmd_vel_topic_ = this->get_parameter("cmd_vel_ackermann_topic").as_string();
  wheel_base_ = this->get_parameter("wheel_base").as_double();

  // create publisher and subscriber
  subscription_diff_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    diff_cmd_vel_topic_, 10, std::bind(&VelocityCommandsAdapterNode::diff_velocity_command_callback, this, std::placeholders::_1));
  publisher_ackermann_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(ackermann_cmd_vel_topic_, 10);
}

void VelocityCommandsAdapterNode::diff_velocity_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  geometry_msgs::msg::Twist ackermann_twist_stamped_msg = VelocityCommandsAdapterLib::differential_drive_to_ackermann(*msg, wheel_base_);
  publisher_ackermann_cmd_vel_->publish(ackermann_twist_stamped_msg);
}

}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<velocity_commands_adapter::VelocityCommandsAdapterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
