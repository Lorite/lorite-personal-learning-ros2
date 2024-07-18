#ifndef VELOCITY_COMMANDS_ADAPTER_NODE_HPP
#define VELOCITY_COMMANDS_ADAPTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <velocity_commands_adapter/velocity_commands_adapter_lib.hpp>

namespace velocity_commands_adapter {

  /**
   * @brief Node that contains methods to convert one type of twist commands to another
   * Subscribers:
   * - /cmd_vel_differential_drive_topic (geometry_msgs::msg::Twist): The differential drive twist command. It is converted and published as an ackermann twist command
   * Publishers:
   * - /cmd_vel_ackermann_topic (geometry_msgs::msg::Twist): The ackermann twist command. It is published after converting the differential drive twist command.
   * Parameters:
   * - wheel_base: The distance between the front and rear axles of the robot
   * - diff_cmd_vel_topic: The topic name of the differential drive velocity subscriber
   * - ackermann_cmd_vel_topic: The topic name of the ackermann velocity publisher
   * Units are in SI units
   */
  class VelocityCommandsAdapterNode : public rclcpp::Node {
  public:
    /**
     * @brief Construct a new twist commands Adapter Node object
     */
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
