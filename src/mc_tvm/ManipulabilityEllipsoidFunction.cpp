/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/ManipulabilityEllipsoidFunction.h>

#include <mc_tvm/Robot.h>

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotFrame.h>

#include <Tasks/SPDManifold.h>

namespace mc_tvm
{

namespace
{

/** Size of the vectorized ellipsoid, at least 1 (invalid axes are reported by tasks::ManipulabilityEllipsoidTask) */
int ellipsoidVecSize(const Eigen::Vector6d & axes)
{
  return std::max(tasks::spd::vecSize(static_cast<int>((axes.array() != 0.).count())), 1);
}

} // namespace

ManipulabilityEllipsoidFunction::ManipulabilityEllipsoidFunction(const mc_rbdyn::RobotFrame & frame,
                                                                 const std::vector<std::string> & measureJoints,
                                                                 const Eigen::Vector6d & axes,
                                                                 tasks::ManipulabilityEllipsoidType type)
: tvm::function::abstract::Function(ellipsoidVecSize(axes)), frame_(frame),
  ellipsoid_(frame.robot().mb(), frame.body(), frame.X_b_f(), measureJoints, axes, type),
  refVel_(Eigen::VectorXd::Zero(size())), refAccel_(Eigen::VectorXd::Zero(size()))
{
  reset();
  // clang-format off
  registerUpdates(Update::Value, &ManipulabilityEllipsoidFunction::updateValue,
                  Update::Velocity, &ManipulabilityEllipsoidFunction::updateVelocity,
                  Update::Jacobian, &ManipulabilityEllipsoidFunction::updateJacobian,
                  Update::NormalAcceleration, &ManipulabilityEllipsoidFunction::updateNormalAcceleration);
  // clang-format on
  addOutputDependency<ManipulabilityEllipsoidFunction>(Output::Value, Update::Value);
  addOutputDependency<ManipulabilityEllipsoidFunction>(Output::Velocity, Update::Velocity);
  addOutputDependency<ManipulabilityEllipsoidFunction>(Output::Jacobian, Update::Jacobian);
  addOutputDependency<ManipulabilityEllipsoidFunction>(Output::NormalAcceleration, Update::NormalAcceleration);
  auto & tvm_robot = frame.robot().tvmRobot();
  addVariable(tvm_robot.q(), false);
  // tasks::ManipulabilityEllipsoidTask computes every output at once from the configuration and velocity of the robot
  addInputDependency<ManipulabilityEllipsoidFunction>(Update::Value, tvm_robot, mc_tvm::Robot::Output::FK);
  addInputDependency<ManipulabilityEllipsoidFunction>(Update::Value, tvm_robot, mc_tvm::Robot::Output::FV);
  addInternalDependency<ManipulabilityEllipsoidFunction>(Update::Velocity, Update::Value);
  addInternalDependency<ManipulabilityEllipsoidFunction>(Update::Jacobian, Update::Value);
  addInternalDependency<ManipulabilityEllipsoidFunction>(Update::NormalAcceleration, Update::Value);
}

void ManipulabilityEllipsoidFunction::reset()
{
  const auto & robot = frame_->robot();
  ellipsoid_.update(robot.mb(), robot.mbc());
  ellipsoid_.resetTarget();
  refVel_.setZero();
  refAccel_.setZero();
}

void ManipulabilityEllipsoidFunction::updateValue()
{
  const auto & robot = frame_->robot();
  ellipsoid_.update(robot.mb(), robot.mbc());
  value_ = -ellipsoid_.eval();
}

void ManipulabilityEllipsoidFunction::updateVelocity()
{
  velocity_ = ellipsoid_.speed() - refVel_;
}

void ManipulabilityEllipsoidFunction::updateJacobian()
{
  splitJacobian(ellipsoid_.jac(), frame_->robot().tvmRobot().q());
}

void ManipulabilityEllipsoidFunction::updateNormalAcceleration()
{
  normalAcceleration_ = ellipsoid_.normalAcc() - refAccel_;
}

} // namespace mc_tvm
