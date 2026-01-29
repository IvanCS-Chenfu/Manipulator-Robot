cmake_minimum_required(VERSION 3.5)
project(copy_six)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# ——— Install your URDF & launch files into share/ ———
install(
  DIRECTORY
    urdf
    launch
  DESTINATION share/${PROJECT_NAME}
)

# ——— Build & install your C++ nodes into lib/<pkg>/ ———

add_executable(inverse_dynamics_controller_node
  src/inverse_dynamics_controller_node.cpp
)
ament_target_dependencies(inverse_dynamics_controller_node
  rclcpp std_msgs
)
target_link_libraries(inverse_dynamics_controller_node
  Eigen3::Eigen
)
install(TARGETS inverse_dynamics_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# ——— Install your Python node into libexec/ so ROS2 can find it ———
install(PROGRAMS
  scripts/dynamics_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()