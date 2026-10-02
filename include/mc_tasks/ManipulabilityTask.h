/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <Tasks/Tasks.h>

namespace mc_tasks
{

/*! \brief Control the manipulability of a frame
 *
 * The task drives the manipulability \f$ w(q) \f$ of a frame toward a target, as a one-dimensional trajectory task.
 * The measure is Yoshikawa's velocity manipulability \f$ w = \sqrt{\det(\tilde{J}\tilde{J}^T)} \f$ computed on the
 * frame Jacobian in frame coordinates, restricted to some frame axes and measure joints, and optionally normalized by
 * the maximum frame and joint velocities (see tasks::ManipulabilityTask).
 *
 * reset() sets the target to the current manipulability. To maximize the manipulability, set a target above the
 * reachable values or a positive reference velocity. The measure joints only define the measure, use
 * selectActiveJoints() to choose the joints used to change it.
 */
struct MC_TASKS_DLLAPI ManipulabilityTask : public TrajectoryTaskGeneric
{
  /*! \brief Constructor
   *
   * \param frame Frame whose manipulability is controlled
   *
   * \param measureJoints Joints whose Jacobian columns define the measure, if empty every joint between the robot
   * root and the frame but a floating base
   *
   * \param axes Frame axes used by the measure, ordered as sva::MotionVecd (rotation then translation), a non-zero
   * entry selects the axis
   *
   * \param measure Manipulability measure
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   * \throws std::domain_error if the measure joints or the axes are invalid
   */
  ManipulabilityTask(const mc_rbdyn::RobotFrame & frame,
                     const std::vector<std::string> & measureJoints = {},
                     const Eigen::Vector6d & axes = Eigen::Vector6d::Ones(),
                     tasks::ManipulabilityMeasure measure = tasks::ManipulabilityMeasure::Yoshikawa,
                     double stiffness = 2.0,
                     double weight = 1000.0);

  /*! \brief Reset the task
   *
   * Set the target to the current manipulability
   */
  void reset() override;

  /*! \brief Set the target manipulability */
  void target(double target);

  /*! \brief Get the target manipulability */
  double target() const;

  /*! \brief Manipulability of the frame, computed at the last solver update */
  double manipulability() const;

  /*! \brief Set the maximum frame velocity along each axis, ordered as the axes (Yoshikawa's \f$ \dot{r}_0 \f$)
   *
   * Entries of non-selected axes are unused
   */
  void maxTaskVelocity(const Eigen::Vector6d & velocity);

  /*! \brief Get the maximum frame velocity along each axis */
  const Eigen::Vector6d & maxTaskVelocity() const;

  /*! \brief Set the maximum velocity of measure joints (Yoshikawa's \f$ \dot{\theta}_0 \f$)
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

  /*! \brief Joints whose Jacobian columns define the measure, ordered from the robot root to the frame */
  const std::vector<std::string> & measureJoints() const;

  /*! \brief Frame axes used by the measure */
  const Eigen::Vector6d & axes() const;

  /*! \brief Manipulability measure */
  tasks::ManipulabilityMeasure measure() const;

  /*! \brief Frame whose manipulability is controlled */
  inline const mc_rbdyn::RobotFrame & frame() const noexcept { return *frame_; }

  /*! \brief Load from configuration
   *
   * Besides the TrajectoryTaskGeneric entries, reads target, maxTaskVelocity, useVelocityLimits and maxJointVelocity
   * (in this order, so that maxJointVelocity overrides the limits of the given joints)
   */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;

  mc_rbdyn::ConstRobotFramePtr frame_;
  /** Manipulability computation, owned by the backend task */
  tasks::ManipulabilityTask * manipulability_ = nullptr;
};

} // namespace mc_tasks
