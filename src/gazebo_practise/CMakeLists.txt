cmake_minimum_required(VERSION 3.8)
project(gazebo_practise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)  # For Gazebo integration
find_package(robot_state_publisher REQUIRED)  # For publishing robot state
find_package(rviz2 REQUIRED)  # For RViz visualization
find_package(franka_description REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(franka_description REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install URDF & sdf files
install(
  DIRECTORY urdf worlds config controller_config
  DESTINATION share/${PROJECT_NAME})

add_library(HybridFTController SHARED controller/robot_controller.cpp)

add_executable(reference_generator src/reference_generator.cpp)

# Link the executable reference_generator with required libraries
ament_target_dependencies(reference_generator
  rclcpp 
  trajectory_msgs 
  sensor_msgs 
  Eigen3 
  kdl_parser
)

# Link the library HybridFTController with required libraries
ament_target_dependencies(HybridFTController
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  Eigen3
)

pluginlib_export_plugin_description_file(controller_interface plugins.xml)

install(TARGETS HybridFTController
  DESTINATION lib)

ament_package()
