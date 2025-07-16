cmake_minimum_required(VERSION 3.5)
project(six_dof_manipulator_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)

# Install URDF and launch folders
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
 src/dynamics_printer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()