#ifndef VELOCITY_COMMANDS_ADAPTER_DIFFERENTIAL_DRIVE_TO_ACKERMANN_CPP
#define VELOCITY_COMMANDS_ADAPTER_DIFFERENTIAL_DRIVE_TO_ACKERMANN_CPP

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>

namespace velocity_commands_adapter {

  /**
   * @brief Class that contains methods to convert one type of twist commands to another
   * For now it only contains a method to convert differential drive twist commands to ackermann twist commands
   */
  class VelocityCommandsAdapterLib {
  public:
    /**
     * @brief Converts a differential drive twist command to an ackermann twist command
     * 
     * @param differential_drive_twist_stamped_msg The differential drive twist command to be converted
     * @return geometry_msgs::msg::Twist The ackermann twist command
     */
    [[nodiscard]] static geometry_msgs::msg::Twist differential_drive_to_ackermann(
        const geometry_msgs::msg::Twist& differential_drive_twist_stamped_msg,
        double wheel_base);
          
  protected:
    /**
     * @brief Converts an angular velocity to a steering angle
     * 
     * @param linear_velocity The linear velocity of the robot
     * @param angular_velocity The angular velocity of the robot
     * @return double The steering angle
     */
    [[nodiscard]] static double angular_velocity_to_steering_angle(double linear_velocity, double angular_velocity, double wheel_base);
  };
    
}

#endif // VELOCITY_COMMANDS_ADAPTER_DIFFERENTIAL_DRIVE_TO_ACKERMANN_CPP
