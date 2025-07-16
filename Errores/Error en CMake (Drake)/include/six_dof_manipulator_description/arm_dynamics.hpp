#pragma once

#include <string>
#include <Eigen/Dense>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsers/urdf_parser.h>

namespace six_dof_manipulator_description {

class ArmDynamics {
 public:
  ArmDynamics(const std::string& urdf_path, const std::string& ee_frame);
  void Compute(const Eigen::VectorXd& q,
               const Eigen::VectorXd& v,
               Eigen::MatrixXd& H,
               Eigen::VectorXd& b,
               Eigen::MatrixXd& J,
               Eigen::MatrixXd& Jdot);

 private:
  drake::multibody::MultibodyPlant<double> plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const drake::multibody::Frame<double>* ee_frame_;
};

}  // namespace six_dof_manipulator_description
