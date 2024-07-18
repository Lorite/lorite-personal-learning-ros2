#include <velocity_commands_adapter/velocity_commands_adapter_lib.hpp>

namespace velocity_commands_adapter {

  geometry_msgs::msg::Twist VelocityCommandsAdapterLib::differential_drive_to_ackermann(
      const geometry_msgs::msg::Twist& differential_drive_twist_stamped_msg,
      double wheel_base) {
    geometry_msgs::msg::Twist ackermann_twist_stamped_msg;
    ackermann_twist_stamped_msg.linear.x = differential_drive_twist_stamped_msg.linear.x;
    ackermann_twist_stamped_msg.angular.z = angular_velocity_to_steering_angle(
      ackermann_twist_stamped_msg.linear.x,
      differential_drive_twist_stamped_msg.angular.z,
      wheel_base);
    return ackermann_twist_stamped_msg;
  }

  double VelocityCommandsAdapterLib::angular_velocity_to_steering_angle(double linear_velocity, double angular_velocity, double wheel_base) {
    if (linear_velocity == 0) {
      return 0;
    } else {
      return atan(angular_velocity * wheel_base / linear_velocity);
    }
  }

}
