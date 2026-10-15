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

/** Maximum joint velocities of the measure joints of the manipulability tasks (ManipulabilityTask and
 * ManipulabilityEllipsoidTask), ordered as the measure joints degrees of freedom */
namespace mc_tasks::details
{

/** Set the maximum velocity of some measure joints, for every degree of freedom of each joint
 *
 * \throws std::domain_error if a joint is not a measure joint
 */
inline Eigen::VectorXd maxJointVelocity(const mc_rbdyn::Robot & robot,
                                        const std::vector<std::string> & measureJoints,
                                        Eigen::VectorXd maxVelocity,
                                        const std::map<std::string, double> & velocities,
                                        const std::string & taskName)
{
  for(const auto & [name, velocity] : velocities)
  {
    Eigen::Index start = 0;
    bool found = false;
    for(const auto & j : measureJoints)
    {
      auto dof = robot.mb().joint(static_cast<int>(robot.jointIndexByName(j))).dof();
      if(j == name)
      {
        maxVelocity.segment(start, dof).setConstant(velocity);
        found = true;
        break;
      }
      start += dof;
    }
    if(!found) { throw std::domain_error(fmt::format("[{}] {} is not a measure joint of this task", taskName, name)); }
  }
  return maxVelocity;
}

/** Maximum velocity of every measure joint from the robot velocity limits, the smallest of the lower and upper limit
 * magnitudes
 *
 * \throws std::domain_error if a measure joint has no finite and non-zero velocity limit
 */
inline Eigen::VectorXd maxJointVelocityFromLimits(const mc_rbdyn::Robot & robot,
                                                  const std::vector<std::string> & measureJoints,
                                                  Eigen::VectorXd maxVelocity,
                                                  const std::string & taskName)
{
  Eigen::Index start = 0;
  for(const auto & j : measureJoints)
  {
    auto jIndex = robot.jointIndexByName(j);
    const auto & vl = robot.vl()[jIndex];
    const auto & vu = robot.vu()[jIndex];
    for(size_t d = 0; d < vu.size(); ++d)
    {
      double velocity = std::min(std::abs(vl[d]), std::abs(vu[d]));
      if(!std::isfinite(velocity) || velocity <= 0.)
      {
        throw std::domain_error(fmt::format("[{}] {} has no usable velocity limit", taskName, j));
      }
      maxVelocity(start++) = velocity;
    }
  }
  return maxVelocity;
}

} // namespace mc_tasks::details
