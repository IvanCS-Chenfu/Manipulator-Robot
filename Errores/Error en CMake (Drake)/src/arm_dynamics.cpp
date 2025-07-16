#include "six_dof_manipulator_description/arm_dynamics.hpp"
#include <drake/math/autodiff.h>
#include <drake/multibody/plant/joint_actuator.h>

namespace six_dof_manipulator_description {

using namespace drake::multibody;
using drake::systems::Context;
using drake::math::JacobianWrtVariable;
using VectorX = Eigen::VectorXd;

ArmDynamics::ArmDynamics(const std::string& urdf_path,
                         const std::string& ee_frame_name)
  : plant_{0.0},
    context_{plant_.CreateDefaultContext()} 
{
  parsers::urdf::AddModelFromFile(urdf_path, &plant_);
  plant_.Finalize();
  ee_frame_ = &plant_.GetFrameByName(ee_frame_name);
}

void ArmDynamics::Compute(const VectorX& q,
                          const VectorX& v,
                          Eigen::MatrixXd& H,
                          VectorX& b,
                          Eigen::MatrixXd& J,
                          Eigen::MatrixXd& Jdot) 
{
  plant_.SetPositions(context_.get(), q);
  plant_.SetVelocities(context_.get(), v);

  // mass matrix
  H = plant_.CalcMassMatrixViaInverseDynamics(*context_);

  // bias = C + g
  auto zero_acc = VectorX::Zero(plant_.num_velocities());
  auto zero_tau = VectorX::Zero(plant_.num_velocities());
  b = plant_.CalcInverseDynamics(*context_, zero_tau, zero_acc);

  // Jacobian & Jacobian derivative
  J    = plant_.CalcJacobianSpatialVelocity(
             *context_,
             JacobianWrtVariable::kQDot,
             plant_.world_frame(),
             Eigen::Vector3d::Zero(),
             *ee_frame_,
             plant_.world_frame());
  Jdot = plant_.CalcJacobianSpatialVelocityTimeDerivative(
             *context_,
             JacobianWrtVariable::kQDot,
             plant_.world_frame(),
             Eigen::Vector3d::Zero(),
             *ee_frame_,
             plant_.world_frame());
}

}  // namespace six_dof_manipulator_description
