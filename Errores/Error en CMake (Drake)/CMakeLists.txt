cmake_minimum_required(VERSION 3.5)
project(six_dof_manipulator_description LANGUAGES CXX)

# — Use C++14 —
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# — Find ROS2 dependencies —
find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(sensor_msgs    REQUIRED)
find_package(std_msgs       REQUIRED)
find_package(xacro          REQUIRED)
find_package(urdf           REQUIRED)
find_package(gazebo_ros     REQUIRED)

# — Find Drake & Eigen (non‑ROS) —
find_package(drake  REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)

# — Expose your headers + Drake’s public headers —
#   so both  #include "six_dof_manipulator_description/arm_dynamics.hpp"
#   and  #include <drake/...>  work.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<TARGET_PROPERTY:drake::common,INTERFACE_INCLUDE_DIRECTORIES>
)

# — Install URDF & launch files —
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

# — Build the arm_dynamics library —  
add_library(arm_dynamics
  src/arm_dynamics.cpp
)

# Link Drake & Eigen into arm_dynamics (propagates include dirs & flags)
target_link_libraries(arm_dynamics
  drake::common
  drake::multibody_plant
  drake::multibody_parsers
  Eigen3::Eigen
)

# — Build the drake_dynamics_node executable —  
add_executable(drake_dynamics_node
  src/drake_dynamics_node.cpp
)

# Link in your library (plain form—only one call!)
target_link_libraries(drake_dynamics_node
  arm_dynamics
)

# ROS‑style dependencies for the node
ament_target_dependencies(drake_dynamics_node
  rclcpp
  sensor_msgs
  std_msgs
  xacro
  urdf
  gazebo_ros
)

# — Install both targets —  
install(
  TARGETS
    arm_dynamics
    drake_dynamics_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
