/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/Function.h>

#include <Tasks/Tasks.h>

namespace mc_tvm
{

/** Difference between the manipulability of a frame and a target manipulability
 *
 * The manipulability and its derivatives are computed by tasks::ManipulabilityTask, see its documentation for the
 * definition of the measure.
 */
class MC_TVM_DLLAPI ManipulabilityFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(ManipulabilityFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * Set the objective to the current manipulability
   *
   * \param frame Frame whose manipulability is measured
   *
   * \param measureJoints Joints whose Jacobian columns define the measure, if empty every joint between the robot
   * root and the frame but a floating base
   *
   * \param axes Frame axes used by the measure, ordered as sva::MotionVecd (rotation then translation), a non-zero
   * entry selects the axis
   *
   * \param measure Manipulability measure
   */
  ManipulabilityFunction(const mc_rbdyn::RobotFrame & frame,
                         const std::vector<std::string> & measureJoints = {},
                         const Eigen::Vector6d & axes = Eigen::Vector6d::Ones(),
                         tasks::ManipulabilityMeasure measure = tasks::ManipulabilityMeasure::Yoshikawa);

  /** Set the target to the current manipulability */
  void reset();

  /** Get the target manipulability */
  inline double target() const noexcept { return manipulability_.target(); }

  /** Set the target manipulability */
  inline void target(double target) noexcept { manipulability_.target(target); }

  /** Manipulability computed by the last value update */
  inline double manipulability() const noexcept { return manipulability_.manipulability(); }

  /** Access the manipulability computation, e.g. to set the velocity normalization */
  inline tasks::ManipulabilityTask & manipulabilityTask() noexcept { return manipulability_; }

  /** Access the manipulability computation */
  inline const tasks::ManipulabilityTask & manipulabilityTask() const noexcept { return manipulability_; }

  /** Get the reference velocity */
  inline const Eigen::VectorXd & refVel() const noexcept { return refVel_; }

  /** Set the reference velocity */
  inline void refVel(const Eigen::VectorXd & refVel) noexcept { refVel_ = refVel; }

  /** Get the reference acceleration */
  inline const Eigen::VectorXd & refAccel() const noexcept { return refAccel_; }

  /** Set the reference acceleration */
  inline void refAccel(const Eigen::VectorXd & refAccel) noexcept { refAccel_ = refAccel; }

  /** Access the frame */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept { return *frame_; }

protected:
  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_rbdyn::ConstRobotFramePtr frame_;

  tasks::ManipulabilityTask manipulability_;

  Eigen::VectorXd refVel_;
  Eigen::VectorXd refAccel_;
};

} // namespace mc_tvm
