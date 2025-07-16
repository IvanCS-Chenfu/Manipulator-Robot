#include "six_dof_manipulator_description/arm_dynamics.hpp"
// … other includes …

using namespace six_dof_manipulator_description;

class DynamicsNode : public rclcpp::Node {
  // …
  ArmDynamics arm_{  // point at your xacro/URDF & ee_frame name
    this->declare_parameter("urdf_path").get<std::string>(),
    this->declare_parameter("ee_frame").get<std::string>()
  };
  // …
  void on_joint_state(...) {
    // build q, v
    Eigen::MatrixXd H, J, Jdot;
    Eigen::VectorXd b;
    arm_.Compute(q, v, H, b, J, Jdot);
    // publish…
  }
  // …
};
