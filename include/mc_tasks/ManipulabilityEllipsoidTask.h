/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <Tasks/Tasks.h>

namespace mc_tasks
{

/*! \brief Track a manipulability ellipsoid of a frame
 *
 * The task drives the manipulability ellipsoid \f$ M(q) \f$ of a frame, a symmetric positive definite (SPD) matrix,
 * toward a target ellipsoid \f$ \hat{M} \f$ with the geometry-aware error \f$ \mathrm{Log}_M(\hat{M}) \f$ on the
 * manifold of SPD matrices (Jaquier et al., "Geometry-aware manipulability learning, tracking, and transfer", IJRR
 * 2021). With the trajectory task dynamics this is the acceleration-based tracking controller of that article (eqs. 46
 * and 47): the stiffness and damping are its gains, the target velocity and acceleration are the task reference
 * velocity and acceleration.
 *
 * The ellipsoid is the velocity manipulability \f$ M = \tilde{J}\tilde{J}^T \f$ of the frame Jacobian expressed in
 * world coordinates, restricted to some world axes (translation by default) and measure joints, with its columns
 * optionally scaled by the maximum joint velocities (see tasks::ManipulabilityEllipsoidTask).
 *
 * For D selected axes the task dimension is D (D + 1) / 2: matrices are vectorized with the Mandel notation (see
 * tasks::spd::vec), so dimWeight and dimStiffness apply to the diagonal entries first, then to each superdiagonal.
 *
 * reset() sets the target to the current ellipsoid. The measure joints only define the ellipsoid, use
 * selectActiveJoints() to choose the joints used to change it.
 */
struct MC_TASKS_DLLAPI ManipulabilityEllipsoidTask : public TrajectoryTaskGeneric
{
  /*! \brief Constructor
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
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \throws std::domain_error if the measure joints or the axes are invalid
   */
  ManipulabilityEllipsoidTask(const mc_rbdyn::RobotFrame & frame,
                              const std::vector<std::string> & measureJoints = {},
                              const Eigen::Vector6d & axes = (Eigen::Vector6d() << 0., 0., 0., 1., 1., 1.).finished(),
                              tasks::ManipulabilityEllipsoidType type = tasks::ManipulabilityEllipsoidType::Velocity,
                              double stiffness = 5.0,
                              double weight = 100.0);

  /*! \brief Reset the task
   *
   * Set the target to the current ellipsoid and the target velocity and acceleration to zero
   */
  void reset() override;

  /*! \brief Set the target ellipsoid
   *
   * \throws std::domain_error if the target is not a size() x size() symmetric positive definite matrix
   */
  void target(const Eigen::MatrixXd & target);

  /*! \brief Get the target ellipsoid */
  const Eigen::MatrixXd & target() const;

  /*! \brief Set the target velocity, a symmetric size() x size() matrix (the task reference velocity) */
  void targetVel(const Eigen::MatrixXd & velocity);

  /*! \brief Set the target acceleration, a symmetric size() x size() matrix (the task reference acceleration) */
  void targetAccel(const Eigen::MatrixXd & acceleration);

  /*! \brief Set the target to the ellipsoid of the robot in another posture
   *
   * Joints that are not given keep their current value. The posture is only used to compute the target.
   *
   * \throws std::domain_error if a joint does not exist or a value has the wrong size
   */
  void targetFromPosture(const std::map<std::string, std::vector<double>> & posture);

  /*! \brief Ellipsoid of the frame, computed at the last solver update */
  const Eigen::MatrixXd & ellipsoid() const;

  /*! \brief Affine-invariant distance between the ellipsoid and the target, computed at the last solver update */
  double distance() const;

  /*! \brief Set the error between the ellipsoid and the target (geometry-aware by default) */
  void error(tasks::ManipulabilityEllipsoidError error);

  /*! \brief Get the error between the ellipsoid and the target */
  tasks::ManipulabilityEllipsoidError error() const;

  /*! \brief Set the maximum velocity of measure joints (Jaquier et al., eq. 48)
   *
   * The velocity applies to every degree of freedom of the joint, joints that are not given keep their value
   *
   * \throws std::domain_error if a joint is not a measure joint or a velocity is not strictly positive
   */
  void maxJointVelocity(const std::map<std::string, double> & velocities);

  /*! \brief Set the maximum velocity of every measure joint from the robot velocity limits
   *
   * The smallest of the lower and upper limit magnitudes is used
   *
   * \throws std::domain_error if a measure joint has no finite and non-zero velocity limit
   */
  void maxJointVelocityFromLimits();

  /*! \brief Maximum velocity of each measure joint degree of freedom, ordered as measureJoints() */
  const Eigen::VectorXd & maxJointVelocity() const;

  /*! \brief Joints whose Jacobian columns define the ellipsoid, ordered from the robot root to the frame */
  const std::vector<std::string> & measureJoints() const;

  /*! \brief World axes of the ellipsoid */
  const Eigen::Vector6d & axes() const;

  /*! \brief Manipulability ellipsoid (type() is the task type) */
  tasks::ManipulabilityEllipsoidType ellipsoidType() const;

  /*! \brief Size of the ellipsoid matrix, the number of selected axes */
  int size() const;

  /*! \brief Frame whose manipulability ellipsoid is controlled */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept { return *frame_; }

  /*! \brief Load from configuration
   *
   * Besides the TrajectoryTaskGeneric entries, reads error, useVelocityLimits, maxJointVelocity, then the target
   * (target or targetPosture, otherwise the current ellipsoid with the new joint velocities), targetVel and
   * targetAccel
   */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;

  /** Compute the ellipsoid in the current configuration and set the target to it */
  void targetCurrent();

  mc_rbdyn::ConstRobotFramePtr frame_;
  /** Ellipsoid computation, owned by the backend task */
  tasks::ManipulabilityEllipsoidTask * ellipsoid_ = nullptr;
  /** Scale of the ellipsoids drawn in the GUI, the semi-axes are scale times the square roots of the eigenvalues */
  double displayScale_ = 0.25;
};

} // namespace mc_tasks
