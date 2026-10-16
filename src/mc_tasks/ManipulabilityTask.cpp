/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/ManipulabilityTask.h>

#include "ManipulabilityJointLimits.h"

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_tvm/ManipulabilityFunction.h>

#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/NumberInput.h>

#include <cmath>

namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<tasks::qp::ManipulabilityTask> tasks_error{};
static inline mc_rtc::void_ptr_caster<mc_tvm::ManipulabilityFunction> tvm_error{};

ManipulabilityTask::ManipulabilityTask(const mc_rbdyn::RobotFrame & frame,
                                       const std::vector<std::string> & measureJoints,
                                       const Eigen::Vector6d & axes,
                                       tasks::ManipulabilityMeasure measure,
                                       double stiffness,
                                       double weight)
: TrajectoryTaskGeneric(frame, stiffness, weight), frame_(frame)
{
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::ManipulabilityTask>(robots.mbs(), static_cast<int>(rIndex), frame.body(),
                                                              frame.X_b_f(), measureJoints, axes, measure);
      manipulability_ = &tasks_error(errorT)->task();
      break;
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::ManipulabilityFunction>(frame, measureJoints, axes, measure);
      manipulability_ = &tvm_error(errorT)->manipulabilityTask();
      break;
    default:
      mc_rtc::log::error_and_throw("[ManipulabilityTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "manipulability";
  name_ = "manipulability_" + frame.robot().name() + "_" + frame.name();
  reset();
}

void ManipulabilityTask::reset()
{
  TrajectoryTaskGeneric::reset();
  auto & manipulability = *manipulability_;
  manipulability.update(frame_->robot().mb(), frame_->robot().mbc());
  manipulability.target(manipulability.manipulability());
}

void ManipulabilityTask::target(double target)
{
  manipulability_->target(target);
}

double ManipulabilityTask::target() const
{
  return manipulability_->target();
}

double ManipulabilityTask::manipulability() const
{
  return manipulability_->manipulability();
}

void ManipulabilityTask::maxTaskVelocity(const Eigen::Vector6d & velocity)
{
  manipulability_->maxTaskVelocity(velocity);
}

const Eigen::Vector6d & ManipulabilityTask::maxTaskVelocity() const
{
  return manipulability_->maxTaskVelocity();
}

void ManipulabilityTask::maxJointVelocity(const std::map<std::string, double> & velocities)
{
  manipulability_->maxJointVelocity(
      details::measureJointValues(frame_->robot(), measureJoints(), maxJointVelocity(), velocities, name_));
}

void ManipulabilityTask::maxJointVelocityFromLimits()
{
  manipulability_->maxJointVelocity(
      details::maxJointVelocityFromLimits(frame_->robot(), measureJoints(), maxJointVelocity(), name_));
}

const Eigen::VectorXd & ManipulabilityTask::maxJointVelocity() const
{
  return manipulability_->maxJointVelocity();
}

const std::vector<std::string> & ManipulabilityTask::measureJoints() const
{
  return manipulability_->measureJoints();
}

const Eigen::Vector6d & ManipulabilityTask::axes() const
{
  return manipulability_->axes();
}

tasks::ManipulabilityMeasure ManipulabilityTask::measure() const
{
  return manipulability_->measure();
}

void ManipulabilityTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  TrajectoryTaskGeneric::load(solver, config);
  if(config.has("target")) { target(static_cast<double>(config("target"))); }
  if(config.has("maxTaskVelocity")) { maxTaskVelocity(config("maxTaskVelocity")); }
  if(config("useVelocityLimits", false)) { maxJointVelocityFromLimits(); }
  if(config.has("maxJointVelocity")) { maxJointVelocity(config("maxJointVelocity")); }
}

void ManipulabilityTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryTaskGeneric::addToLogger(logger);
  MC_RTC_LOG_GETTER(name_ + "_manipulability", manipulability);
  MC_RTC_LOG_GETTER(name_ + "_target", target);
}

void ManipulabilityTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric::addToGUI(gui);
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Label("manipulability", [this]() { return manipulability(); }),
                 mc_rtc::gui::NumberInput("target", [this]() { return target(); }, [this](double t) { target(t); }));
}

} // namespace mc_tasks

namespace
{

tasks::ManipulabilityMeasure measureFromConfig(const mc_rtc::Configuration & config)
{
  std::string measure = config("measure", std::string{"yoshikawa"});
  if(measure == "yoshikawa") { return tasks::ManipulabilityMeasure::Yoshikawa; }
  mc_rtc::log::error_and_throw("[manipulability] Unknown measure {}, supported measures: yoshikawa", measure);
}

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "manipulability",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      const auto & robot = robotFromConfig(config, solver.robots(), "manipulability");
      auto t = std::make_shared<mc_tasks::ManipulabilityTask>(
          robot.frame(config("frame")), config("joints", std::vector<std::string>{}),
          config("axes", Eigen::Vector6d::Ones().eval()), measureFromConfig(config));
      t->load(solver, config);
      return t;
    });

} // namespace
