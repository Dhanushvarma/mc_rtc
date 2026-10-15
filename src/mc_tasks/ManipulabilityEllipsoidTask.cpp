/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/ManipulabilityEllipsoidTask.h>

#include "ManipulabilityJointVelocity.h"

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_tvm/ManipulabilityEllipsoidFunction.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Ellipsoid.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <Tasks/SPDManifold.h>

#include <Eigen/Eigenvalues>

namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<tasks::qp::ManipulabilityEllipsoidTask> tasks_error{};
static inline mc_rtc::void_ptr_caster<mc_tvm::ManipulabilityEllipsoidFunction> tvm_error{};

namespace
{

/** Principal axes (as a rotation) and diameters of the ellipsoid of a 3 x 3 SPD matrix, the semi-axes being scale
 * times the square roots of its eigenvalues */
std::pair<Eigen::Matrix3d, Eigen::Vector3d> ellipsoidShape(const Eigen::MatrixXd & M, double scale)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(Eigen::Matrix3d(M.topLeftCorner<3, 3>()));
  Eigen::Matrix3d V = es.eigenvectors();
  if(V.determinant() < 0) { V.col(0) *= -1; }
  Eigen::Vector3d size = (2 * scale * es.eigenvalues().cwiseMax(0.).cwiseSqrt()).cwiseMax(1e-3);
  return {V, size};
}

} // namespace

ManipulabilityEllipsoidTask::ManipulabilityEllipsoidTask(const mc_rbdyn::RobotFrame & frame,
                                                         const std::vector<std::string> & measureJoints,
                                                         const Eigen::Vector6d & axes,
                                                         tasks::ManipulabilityEllipsoidType type,
                                                         double stiffness,
                                                         double weight)
: TrajectoryTaskGeneric(frame, stiffness, weight), frame_(frame)
{
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::ManipulabilityEllipsoidTask>(
          robots.mbs(), static_cast<int>(rIndex), frame.body(), frame.X_b_f(), measureJoints, axes, type);
      ellipsoid_ = &tasks_error(errorT)->task();
      break;
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::ManipulabilityEllipsoidFunction>(frame, measureJoints, axes, type);
      ellipsoid_ = &tvm_error(errorT)->ellipsoidTask();
      break;
    default:
      mc_rtc::log::error_and_throw("[ManipulabilityEllipsoidTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "manipulabilityEllipsoid";
  name_ = "manipulability_ellipsoid_" + frame.robot().name() + "_" + frame.name();
  reset();
}

void ManipulabilityEllipsoidTask::reset()
{
  TrajectoryTaskGeneric::reset();
  targetCurrent();
}

void ManipulabilityEllipsoidTask::targetCurrent()
{
  const auto & robot = frame_->robot();
  ellipsoid_->update(robot.mb(), robot.mbc());
  ellipsoid_->resetTarget();
}

void ManipulabilityEllipsoidTask::target(const Eigen::MatrixXd & target)
{
  ellipsoid_->target(target);
}

const Eigen::MatrixXd & ManipulabilityEllipsoidTask::target() const
{
  return ellipsoid_->target();
}

void ManipulabilityEllipsoidTask::targetVel(const Eigen::MatrixXd & velocity)
{
  if(velocity.rows() != size() || velocity.cols() != size())
  {
    throw std::domain_error(fmt::format("[{}] the target velocity must be a {}x{} matrix", name_, size(), size()));
  }
  refVel(tasks::spd::vec(velocity));
}

void ManipulabilityEllipsoidTask::targetAccel(const Eigen::MatrixXd & acceleration)
{
  if(acceleration.rows() != size() || acceleration.cols() != size())
  {
    throw std::domain_error(fmt::format("[{}] the target acceleration must be a {}x{} matrix", name_, size(), size()));
  }
  refAccel(tasks::spd::vec(acceleration));
}

void ManipulabilityEllipsoidTask::targetFromPosture(const std::map<std::string, std::vector<double>> & posture)
{
  const auto & robot = frame_->robot();
  rbd::MultiBodyConfig mbc = robot.mbc();
  for(const auto & [name, q] : posture)
  {
    if(!robot.hasJoint(name)) { throw std::domain_error(fmt::format("[{}] no joint named {}", name_, name)); }
    auto jIndex = robot.jointIndexByName(name);
    if(q.size() != mbc.q[jIndex].size())
    {
      throw std::domain_error(
          fmt::format("[{}] joint {} has {} parameters, {} given", name_, name, mbc.q[jIndex].size(), q.size()));
    }
    mbc.q[jIndex] = q;
  }
  rbd::forwardKinematics(robot.mb(), mbc);
  rbd::forwardVelocity(robot.mb(), mbc);
  tasks::ManipulabilityEllipsoidTask ellipsoid = *ellipsoid_;
  ellipsoid.update(robot.mb(), mbc);
  target(ellipsoid.ellipsoid());
}

const Eigen::MatrixXd & ManipulabilityEllipsoidTask::ellipsoid() const
{
  return ellipsoid_->ellipsoid();
}

double ManipulabilityEllipsoidTask::distance() const
{
  return ellipsoid_->distance();
}

void ManipulabilityEllipsoidTask::error(tasks::ManipulabilityEllipsoidError error)
{
  ellipsoid_->error(error);
}

tasks::ManipulabilityEllipsoidError ManipulabilityEllipsoidTask::error() const
{
  return ellipsoid_->error();
}

void ManipulabilityEllipsoidTask::maxJointVelocity(const std::map<std::string, double> & velocities)
{
  ellipsoid_->maxJointVelocity(
      details::maxJointVelocity(frame_->robot(), measureJoints(), maxJointVelocity(), velocities, name_));
}

void ManipulabilityEllipsoidTask::maxJointVelocityFromLimits()
{
  ellipsoid_->maxJointVelocity(
      details::maxJointVelocityFromLimits(frame_->robot(), measureJoints(), maxJointVelocity(), name_));
}

const Eigen::VectorXd & ManipulabilityEllipsoidTask::maxJointVelocity() const
{
  return ellipsoid_->maxJointVelocity();
}

const std::vector<std::string> & ManipulabilityEllipsoidTask::measureJoints() const
{
  return ellipsoid_->measureJoints();
}

const Eigen::Vector6d & ManipulabilityEllipsoidTask::axes() const
{
  return ellipsoid_->axes();
}

tasks::ManipulabilityEllipsoidType ManipulabilityEllipsoidTask::ellipsoidType() const
{
  return ellipsoid_->type();
}

int ManipulabilityEllipsoidTask::size() const
{
  return ellipsoid_->size();
}

void ManipulabilityEllipsoidTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryTaskGeneric::load(solver, config);
  if(config.has("error"))
  {
    std::string errorName = config("error");
    if(errorName == "geometryAware") { error(tasks::ManipulabilityEllipsoidError::GeometryAware); }
    else if(errorName == "euclidean") { error(tasks::ManipulabilityEllipsoidError::Euclidean); }
    else
    {
      mc_rtc::log::error_and_throw("[{}] Unknown error {}, supported errors: geometryAware, euclidean", name_,
                                   errorName);
    }
  }
  bool scaled = false;
  if(config("useVelocityLimits", false))
  {
    maxJointVelocityFromLimits();
    scaled = true;
  }
  if(config.has("maxJointVelocity"))
  {
    maxJointVelocity(config("maxJointVelocity"));
    scaled = true;
  }
  if(config.has("target")) { target(config("target").operator Eigen::MatrixXd()); }
  else if(config.has("targetPosture")) { targetFromPosture(config("targetPosture")); }
  else if(scaled) { targetCurrent(); }
  if(config.has("targetVel")) { targetVel(config("targetVel").operator Eigen::MatrixXd()); }
  if(config.has("targetAccel")) { targetAccel(config("targetAccel").operator Eigen::MatrixXd()); }
}

void ManipulabilityEllipsoidTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryTaskGeneric::addToLogger(logger);
  MC_RTC_LOG_GETTER(name_ + "_distance", distance);
  logger.addLogEntry(name_ + "_ellipsoid", this, [this]() { return tasks::spd::vec(ellipsoid()); });
  logger.addLogEntry(name_ + "_target", this, [this]() { return tasks::spd::vec(target()); });
}

void ManipulabilityEllipsoidTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric::addToGUI(gui);
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Label("distance", [this]() { return distance(); }),
                 mc_rtc::gui::Button("Target = current ellipsoid", [this]() { targetCurrent(); }),
                 mc_rtc::gui::NumberInput(
                     "Display scale", [this]() { return displayScale_; }, [this](double s) { displayScale_ = s; }));
  // Draw the ellipsoids when the axes are the rotation or the translation axes
  const Eigen::Vector6d & a = axes();
  const bool rotation = (a.head<3>().array() != 0.).all() && (a.tail<3>().array() == 0.).all();
  const bool translation = (a.head<3>().array() == 0.).all() && (a.tail<3>().array() != 0.).all();
  if(rotation || translation)
  {
    auto pose = [this](const Eigen::MatrixXd & M)
    {
      return sva::PTransformd(Eigen::Matrix3d(ellipsoidShape(M, displayScale_).first.transpose()),
                              frame_->position().translation());
    };
    gui.addElement({"Tasks", name_},
                   mc_rtc::gui::Ellipsoid(
                       "Ellipsoid", [this]() { return ellipsoidShape(ellipsoid(), displayScale_).second; },
                       [this, pose]() { return pose(ellipsoid()); }, mc_rtc::gui::Color(0.5, 0.2, 0.7, 0.5)),
                   mc_rtc::gui::Ellipsoid(
                       "Target", [this]() { return ellipsoidShape(target(), displayScale_).second; },
                       [this, pose]() { return pose(target()); }, mc_rtc::gui::Color(0.1, 0.7, 0.3, 0.4)));
  }
}

} // namespace mc_tasks

namespace
{

/** Default axes of the ellipsoid, the translation axes */
Eigen::Vector6d translationAxes()
{
  Eigen::Vector6d axes;
  axes << 0., 0., 0., 1., 1., 1.;
  return axes;
}

tasks::ManipulabilityEllipsoidType ellipsoidFromConfig(const mc_rtc::Configuration & config)
{
  std::string ellipsoid = config("ellipsoid", std::string{"velocity"});
  if(ellipsoid == "velocity") { return tasks::ManipulabilityEllipsoidType::Velocity; }
  mc_rtc::log::error_and_throw("[manipulabilityEllipsoid] Unknown ellipsoid {}, supported ellipsoids: velocity",
                               ellipsoid);
}

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "manipulabilityEllipsoid",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto & robot = robotFromConfig(config, solver.robots(), "manipulabilityEllipsoid");
      auto t = std::make_shared<mc_tasks::ManipulabilityEllipsoidTask>(
          robot.frame(config("frame")), config("joints", std::vector<std::string>{}), config("axes", translationAxes()),
          ellipsoidFromConfig(config));
      t->load(solver, config);
      return t;
    });

} // namespace
