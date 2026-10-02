/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ManipulabilityFunction.h>

#include <mc_tvm/Robot.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotFrame.h>

namespace mc_tvm
{

ManipulabilityFunction::ManipulabilityFunction(const mc_rbdyn::RobotFrame & frame,
                                               const std::vector<std::string> & measureJoints,
                                               const Eigen::Vector6d & axes,
                                               tasks::ManipulabilityMeasure measure)
: tvm::function::abstract::Function(1), frame_(frame),
  manipulability_(frame.robot().mb(), frame.body(), frame.X_b_f(), measureJoints, axes, measure),
  refVel_(Eigen::VectorXd::Zero(1)), refAccel_(Eigen::VectorXd::Zero(1))
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &ManipulabilityFunction::updateValue,
                  Update::Velocity, &ManipulabilityFunction::updateVelocity,
                  Update::Jacobian, &ManipulabilityFunction::updateJacobian,
                  Update::NormalAcceleration, &ManipulabilityFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<ManipulabilityFunction>(Output::Value, Update::Value);
  addOutputDependency<ManipulabilityFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<ManipulabilityFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<ManipulabilityFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  auto & tvm_robot = frame.robot().tvmRobot();
  addVariable(tvm_robot.q(), false);
  // tasks::ManipulabilityTask computes every output at once from the configuration and velocity of the robot
  addInputDependency<ManipulabilityFunction>(Update::Value, tvm_robot, mc_tvm::Robot::Output::FK);
  addInputDependency<ManipulabilityFunction>(Update::Value, tvm_robot, mc_tvm::Robot::Output::FV);
  addInternalDependency<ManipulabilityFunction>(Update::Velocity, Update::Value);
  addInternalDependency<ManipulabilityFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<ManipulabilityFunction>(Update::NormalAcceleration, Update::Value);
}

void ManipulabilityFunction::reset()
{
  const auto & robot = frame_->robot();
  manipulability_.update(robot.mb(), robot.mbc());
  manipulability_.target(manipulability_.manipulability());
  refVel_.setZero();
  refAccel_.setZero();
}

void ManipulabilityFunction::updateValue()
{
  const auto & robot = frame_->robot();
  manipulability_.update(robot.mb(), robot.mbc());
  value_(0) = manipulability_.manipulability() - manipulability_.target();
}

void ManipulabilityFunction::updateVelocity()
{
  velocity_ = manipulability_.speed() - refVel_;
}

void ManipulabilityFunction::updateJacobian()
{
  splitJacobian(manipulability_.jac(), frame_->robot().tvmRobot().q());
}

void ManipulabilityFunction::updateNormalAcceleration()
{
  normalAcceleration_ = manipulability_.normalAcc() - refAccel_;
}

} // namespace mc_tvm
