from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  return LaunchDescription([
    DeclareLaunchArgument(
      'wheelbase',
      default_value='0.5',
      description='Wheelbase of the robot in meters'
    ),
    DeclareLaunchArgument(
      'cmd_vel_differential_drive_topic',
      default_value='cmd_vel_differential_drive',
      description='Topic for differential drive commands'
    ),
    DeclareLaunchArgument(
      'cmd_vel_ackermann_topic',
      default_value='cmd_vel_ackermann',
      description='Topic for ackermann drive commands'
    ),
    Node(
      package='velocity_commands_adapter',
      executable='differential_drive_to_ackermann_node',
      name='differential_drive_to_ackermann_node',
      output='screen',
      parameters=[
        {'wheelbase': LaunchConfiguration('wheelbase')},
        {'cmd_vel_differential_drive_topic': LaunchConfiguration('cmd_vel_differential_drive_topic')},
        {'cmd_vel_ackermann_topic': LaunchConfiguration('cmd_vel_ackermann_topic')}
      ]
    )
  ])
