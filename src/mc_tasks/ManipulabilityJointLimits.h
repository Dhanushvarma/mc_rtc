/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/logging.h>

#include <cmath>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

/** Maximum joint velocities and torques of the measure joints of the manipulability tasks (ManipulabilityTask and
 * ManipulabilityEllipsoidTask), ordered as the measure joints degrees of freedom */
namespace mc_tasks::details
{

/** Set the value (maximum velocity or torque) of some measure joints, for every degree of freedom of each joint
 *
 * \throws std::domain_error if a joint is not a measure joint
 */
inline Eigen::VectorXd measureJointValues(const mc_rbdyn::Robot & robot,
                                          const std::vector<std::string> & measureJoints,
                                          Eigen::VectorXd values,
                                          const std::map<std::string, double> & jointValues,
                                          const std::string & taskName)
{
  for(const auto & [name, value] : jointValues)
  {
    Eigen::Index start = 0;
    bool found = false;
    for(const auto & j : measureJoints)
    {
      auto dof = robot.mb().joint(static_cast<int>(robot.jointIndexByName(j))).dof();
      if(j == name)
      {
        values.segment(start, dof).setConstant(value);
        found = true;
        break;
      }
      start += dof;
    }
    if(!found) { throw std::domain_error(fmt::format("[{}] {} is not a measure joint of this task", taskName, name)); }
  }
  return values;
}

/** Limit of every measure joint degree of freedom, the smallest of the lower and upper limit magnitudes
 *
 * \param limit Name of the limit in error messages
 *
 * \throws std::domain_error if a measure joint has no finite and non-zero limit
 */
inline Eigen::VectorXd measureJointLimits(const mc_rbdyn::Robot & robot,
                                          const std::vector<std::string> & measureJoints,
                                          const std::vector<std::vector<double>> & lower,
                                          const std::vector<std::vector<double>> & upper,
                                          Eigen::VectorXd values,
                                          const std::string & limit,
                                          const std::string & taskName)
{
  Eigen::Index start = 0;
  for(const auto & j : measureJoints)
  {
    auto jIndex = robot.jointIndexByName(j);
    const auto & l = lower[jIndex];
    const auto & u = upper[jIndex];
    for(size_t d = 0; d < u.size(); ++d)
    {
      double value = std::min(std::abs(l[d]), std::abs(u[d]));
      if(!std::isfinite(value) || value <= 0.)
      {
        throw std::domain_error(fmt::format("[{}] {} has no usable {} limit", taskName, j, limit));
      }
      values(start++) = value;
    }
  }
  return values;
}

/** Maximum velocity of every measure joint from the robot velocity limits */
inline Eigen::VectorXd maxJointVelocityFromLimits(const mc_rbdyn::Robot & robot,
                                                  const std::vector<std::string> & measureJoints,
                                                  Eigen::VectorXd maxVelocity,
                                                  const std::string & taskName)
{
  return measureJointLimits(robot, measureJoints, robot.vl(), robot.vu(), std::move(maxVelocity), "velocity", taskName);
}

/** Maximum torque of every measure joint from the robot torque limits */
inline Eigen::VectorXd maxJointTorqueFromLimits(const mc_rbdyn::Robot & robot,
                                                const std::vector<std::string> & measureJoints,
                                                Eigen::VectorXd maxTorque,
                                                const std::string & taskName)
{
  return measureJointLimits(robot, measureJoints, robot.tl(), robot.tu(), std::move(maxTorque), "torque", taskName);
}

} // namespace mc_tasks::details
