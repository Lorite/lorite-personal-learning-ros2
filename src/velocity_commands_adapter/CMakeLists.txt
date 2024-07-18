cmake_minimum_required(VERSION 3.8)
project(velocity_commands_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(differential_drive_to_ackermann_node src/differential_drive_to_ackermann_node.cpp src/velocity_commands_adapter_lib.cpp)
ament_target_dependencies(differential_drive_to_ackermann_node rclcpp geometry_msgs)

target_include_directories(differential_drive_to_ackermann_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(differential_drive_to_ackermann_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS differential_drive_to_ackermann_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_velocity_commands_adapter_lib test/test_velocity_commands_adapter_lib.cpp src/velocity_commands_adapter_lib.cpp)
  ament_target_dependencies(test_velocity_commands_adapter_lib rclcpp geometry_msgs)
  target_include_directories(test_velocity_commands_adapter_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  # target_link_libraries(test_velocity_commands_adapter_lib name_of_local_library)
endif()

ament_package()
