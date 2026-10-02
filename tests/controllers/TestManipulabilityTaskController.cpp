/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/ManipulabilityTask.h>
#include <mc_tasks/TransformTask.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

/** The right arm of JVRC-1 has 7 joints: the gripper pose is held while the manipulability task uses the remaining
 * degree of freedom to raise the manipulability of the gripper */
struct MC_CONTROL_DLLAPI TestManipulabilityTaskController : public MCController
{
public:
  TestManipulabilityTaskController(mc_rbdyn::RobotModulePtr rm, double dt, Backend backend)
  : MCController(rm, dt, backend)
  {
    // Check that the default constructor loads the robot + ground environment
    BOOST_CHECK_EQUAL(robots().size(), 2);
    // Check that JVRC-1 was loaded
    BOOST_CHECK_EQUAL(robot().name(), "jvrc1");
    solver().addConstraintSet(contactConstraint);
    solver().addConstraintSet(kinematicsConstraint);
    postureTask->stiffness(2);
    postureTask->weight(1);
    solver().addTask(postureTask.get());
    addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
    addContact({robot().name(), "ground", "RightFoot", "AllGround"});

    gripperTask = std::make_shared<mc_tasks::TransformTask>(robot().frame("RightGripper"), 10.0, 10000.0);
    manipTask = std::make_shared<mc_tasks::ManipulabilityTask>(robot().frame("RightGripper"), rightArm);
    BOOST_CHECK(manipTask->measureJoints() == rightArm);

    mc_rtc::log::success("Created TestManipulabilityTaskController");
  }

  bool run() override
  {
    bool ret = MCController::run();
    if(!ret) { mc_rtc::log::critical("Failed at iter: {}", nrIter); }
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 500)
    {
      // The wrist roll moved the gripper away from the singularity, hold the gripper pose and raise the manipulability
      postureTask->reset();
      gripperTask->reset();
      solver().addTask(gripperTask);
      manipTask->reset();
      w0 = manipTask->manipulability();
      BOOST_CHECK_GT(w0, 1e-3);
      manipTask->target(2 * w0);
      manipTask->selectActiveJoints(solver(), rightArm);
      solver().addTask(manipTask);
    }
    if(nrIter > 500)
    {
      BOOST_CHECK(std::isfinite(manipTask->manipulability()));
      BOOST_CHECK(manipTask->speed().allFinite());
    }
    if(nrIter == 2500)
    {
      double w = manipTask->manipulability();
      mc_rtc::log::info("[TestManipulabilityTaskController] Manipulability {} -> {}", w0, w);
      BOOST_CHECK_GT(w, 1.2 * w0);
      BOOST_CHECK_SMALL(gripperTask->eval().tail<3>().norm(), 5e-3);
      BOOST_CHECK_SMALL(gripperTask->eval().head<3>().norm(), 2e-2);
    }
    return ret;
  }

  void reset(const ControllerResetData & reset_data) override
  {
    MCController::reset(reset_data);
    // With the wrist roll at zero, the elbow and wrist yaw axes are aligned: the gripper manipulability is zero
    manipTask->reset();
    BOOST_CHECK_SMALL(manipTask->manipulability(), 1e-10);
    postureTask->target({{"R_WRIST_R", {0.5}}});
  }

private:
  unsigned int nrIter = 0;
  double w0 = 0.;
  std::vector<std::string> rightArm = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                       "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
  std::shared_ptr<mc_tasks::TransformTask> gripperTask = nullptr;
  std::shared_ptr<mc_tasks::ManipulabilityTask> manipTask = nullptr;
};

} // namespace mc_control

using Controller = mc_control::TestManipulabilityTaskController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestManipulabilityTaskController",
                              Controller(rm, dt, Backend::Tasks),
                              "TestManipulabilityTaskController_TVM",
                              Controller(rm, dt, Backend::TVM))
