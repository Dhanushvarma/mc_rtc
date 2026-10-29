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

/** Error between the manipulability ellipsoid of a frame and a target ellipsoid
 *
 * The value is \f$ -\mathrm{vec}(\mathrm{Log}_M(\hat{M})) \f$ (or \f$ \mathrm{vec}(M - \hat{M}) \f$ with the Euclidean
 * error), vec being the Mandel vectorization. The ellipsoid and its derivatives are computed by
 * tasks::ManipulabilityEllipsoidTask, see its documentation for the definition of the ellipsoid, also relative to a
 * reference frame.
 */
class MC_TVM_DLLAPI ManipulabilityEllipsoidFunction : public tvm::function::abstract::Function
{
public:
  SET_UPDATES(ManipulabilityEllipsoidFunction, Value, Velocity, Jacobian, NormalAcceleration)

  /** Constructor
   *
   * Set the target to the current ellipsoid
   *
   * \param frame Frame whose manipulability ellipsoid is controlled
   *
   * \param measureJoints Joints whose Jacobian columns define the ellipsoid, if empty every joint between the robot
   * root and the frame but a floating base
   *
   * \param axes World axes of the ellipsoid, ordered as sva::MotionVecd (rotation then translation), a non-zero entry
   * selects the axis
   *
   * \param type Manipulability ellipsoid
   */
  ManipulabilityEllipsoidFunction(
      const mc_rbdyn::RobotFrame & frame,
      const std::vector<std::string> & measureJoints = {},
      const Eigen::Vector6d & axes = (Eigen::Vector6d() << 0., 0., 0., 1., 1., 1.).finished(),
      tasks::ManipulabilityEllipsoidType type = tasks::ManipulabilityEllipsoidType::Velocity);

  /** Constructor for the ellipsoid of a frame relative to a reference frame of the same robot
   *
   * Set the target to the current ellipsoid
   *
   * \param frame Frame whose manipulability ellipsoid is controlled
   *
   * \param referenceFrame Reference frame, the ellipsoid describes the motion of the frame relative to it, in its axes
   *
   * \param measureJoints Joints whose Jacobian columns define the ellipsoid, if empty every joint between the two
   * frames: the joints between the robot root and one frame that are not between the robot root and the other one
   *
   * \param axes Reference frame axes of the ellipsoid, ordered as sva::MotionVecd (rotation then translation), a
   * non-zero entry selects the axis
   *
   * \param type Manipulability ellipsoid
   *
   * \throws std::domain_error if the frames belong to different robots
   */
  ManipulabilityEllipsoidFunction(
      const mc_rbdyn::RobotFrame & frame,
      const mc_rbdyn::RobotFrame & referenceFrame,
      const std::vector<std::string> & measureJoints = {},
      const Eigen::Vector6d & axes = (Eigen::Vector6d() << 0., 0., 0., 1., 1., 1.).finished(),
      tasks::ManipulabilityEllipsoidType type = tasks::ManipulabilityEllipsoidType::Velocity);

  /** Set the target to the current ellipsoid and the reference velocity and acceleration to zero */
  void reset();

  /** Access the ellipsoid computation, e.g. to set the target or the error */
  inline tasks::ManipulabilityEllipsoidTask & ellipsoidTask() noexcept { return ellipsoid_; }

  /** Access the ellipsoid computation */
  inline const tasks::ManipulabilityEllipsoidTask & ellipsoidTask() const noexcept { return ellipsoid_; }

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

  /** Reference frame of a relative ellipsoid, nullptr otherwise */
  inline const mc_rbdyn::RobotFrame * referenceFrame() const noexcept { return referenceFrame_.get(); }

protected:
  ManipulabilityEllipsoidFunction(const mc_rbdyn::RobotFrame & frame,
                                  const mc_rbdyn::RobotFrame * referenceFrame,
                                  const std::vector<std::string> & measureJoints,
                                  const Eigen::Vector6d & axes,
                                  tasks::ManipulabilityEllipsoidType type);

  void updateValue();
  void updateVelocity();
  void updateJacobian();
  void updateNormalAcceleration();

  mc_rbdyn::ConstRobotFramePtr frame_;
  mc_rbdyn::ConstRobotFramePtr referenceFrame_;

  tasks::ManipulabilityEllipsoidTask ellipsoid_;

  Eigen::VectorXd refVel_;
  Eigen::VectorXd refAccel_;
};

} // namespace mc_tvm
