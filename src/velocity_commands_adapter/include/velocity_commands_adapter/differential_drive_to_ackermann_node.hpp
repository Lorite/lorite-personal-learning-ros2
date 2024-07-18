#ifndef VELOCITY_COMMANDS_ADAPTER_NODE_HPP
#define VELOCITY_COMMANDS_ADAPTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <velocity_commands_adapter/velocity_commands_adapter_lib.hpp>

namespace velocity_commands_adapter {

  class VelocityCommandsAdapterNode : public rclcpp::Node {
  public:
    VelocityCommandsAdapterNode();

  private:
    void diff_velocity_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void initialize();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_diff_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ackermann_cmd_vel_;
    std::string diff_cmd_vel_topic_;
    std::string ackermann_cmd_vel_topic_;
    double wheel_base_;
  };

}

#endif // VELOCITY_COMMANDS_ADAPTER_NODE_HPP
