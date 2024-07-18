#include <gtest/gtest.h>
#include <velocity_commands_adapter/velocity_commands_adapter_lib.hpp>

class TestVelocityCommandAdapterLib : public ::testing::Test, public velocity_commands_adapter::VelocityCommandsAdapterLib {

void SetUp() override {
}

void TearDown() override {
}

};

TEST_F(TestVelocityCommandAdapterLib, TestAngularVelocityToSteeringAngleWithPositiveValues) {
  double linear_velocity = 5.0;
  double angular_velocity = 2.0;
  double wheel_base = 0.5;
  double steering_angle = velocity_commands_adapter::VelocityCommandsAdapterLib::angular_velocity_to_steering_angle(linear_velocity, angular_velocity, wheel_base);
  EXPECT_NEAR(steering_angle, 0.3805, 0.001);
}

TEST_F(TestVelocityCommandAdapterLib, TestAngularVelocityToSteeringAngleWithNegativeValues) {
  double linear_velocity = 5.0;
  double angular_velocity = -2.0;
  double wheel_base = 0.5;
  double steering_angle = velocity_commands_adapter::VelocityCommandsAdapterLib::angular_velocity_to_steering_angle(linear_velocity, angular_velocity, wheel_base);
  EXPECT_NEAR(steering_angle, -0.3805, 0.001);
}

TEST_F(TestVelocityCommandAdapterLib, TestAngularVelocityToSteeringAngleWithZeroValues) {
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
  double wheel_base = 0.5;
  double steering_angle = velocity_commands_adapter::VelocityCommandsAdapterLib::angular_velocity_to_steering_angle(linear_velocity, angular_velocity, wheel_base);
  EXPECT_EQ(steering_angle, 0.0);
}

TEST_F(TestVelocityCommandAdapterLib, TestDifferentialDriveToAckermann) {
  geometry_msgs::msg::Twist differential_drive_twist_stamped_msg;
  differential_drive_twist_stamped_msg.linear.x = 5.0;
  differential_drive_twist_stamped_msg.angular.z = 2.0;
  double wheel_base = 0.5;
  geometry_msgs::msg::Twist ackermann_twist_stamped_msg = 
    velocity_commands_adapter::VelocityCommandsAdapterLib::differential_drive_to_ackermann(differential_drive_twist_stamped_msg, wheel_base);
  EXPECT_EQ(ackermann_twist_stamped_msg.linear.x, 5.0);
  EXPECT_NEAR(ackermann_twist_stamped_msg.angular.z, 0.3805, 0.001);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  std::cout << "Running tests" << std::endl;
  return RUN_ALL_TESTS();
}