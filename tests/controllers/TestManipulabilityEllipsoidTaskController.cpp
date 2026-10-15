/*
 * Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef BOOST_TEST_MAIN
#  undef BOOST_TEST_MAIN
#endif

#include <mc_control/api.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/ManipulabilityEllipsoidTask.h>

#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

namespace mc_control
{

/** The right arm of JVRC-1 tracks the translation ellipsoid of its gripper in another arm posture */
struct MC_CONTROL_DLLAPI TestManipulabilityEllipsoidTaskController : public MCController
{
public:
  TestManipulabilityEllipsoidTaskController(mc_rbdyn::RobotModulePtr rm, double dt, Backend backend)
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

    ellipsoidTask = std::make_shared<mc_tasks::ManipulabilityEllipsoidTask>(robot().frame("RightGripper"), rightArm);
    BOOST_CHECK_EQUAL(ellipsoidTask->size(), 3);
    BOOST_CHECK_EQUAL(ellipsoidTask->dimWeight().size(), 6);

    mc_rtc::log::success("Created TestManipulabilityEllipsoidTaskController");
  }

  bool run() override
  {
    bool ret = MCController::run();
    if(!ret) { mc_rtc::log::critical("Failed at iter: {}", nrIter); }
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 100)
    {
      // Target: the ellipsoid of the arm moved forward with the elbow bent further
      const auto & q = robot().mbc().q;
      auto jointQ = [&](const std::string & j) { return q[robot().jointIndexByName(j)][0]; };
      ellipsoidTask->reset();
      ellipsoidTask->targetFromPosture({{"R_SHOULDER_P", {jointQ("R_SHOULDER_P") - 0.4}},
                                        {"R_SHOULDER_Y", {jointQ("R_SHOULDER_Y") + 0.3}},
                                        {"R_ELBOW_P", {jointQ("R_ELBOW_P") - 0.4}}});
      ellipsoidTask->selectActiveJoints(solver(), rightArm);
      solver().addTask(ellipsoidTask);
    }
    if(nrIter == 101)
    {
      d0 = ellipsoidTask->distance();
      BOOST_CHECK_GT(d0, 0.1);
    }
    if(nrIter > 100)
    {
      BOOST_CHECK(std::isfinite(ellipsoidTask->distance()));
      BOOST_CHECK(ellipsoidTask->speed().allFinite());
    }
    if(nrIter == 2000)
    {
      double d = ellipsoidTask->distance();
      mc_rtc::log::info("[TestManipulabilityEllipsoidTaskController] Distance to the target {} -> {}", d0, d);
      BOOST_CHECK_LT(d, 0.1 * d0);
    }
    return ret;
  }

private:
  unsigned int nrIter = 0;
  double d0 = 0.;
  std::vector<std::string> rightArm = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                       "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
  std::shared_ptr<mc_tasks::ManipulabilityEllipsoidTask> ellipsoidTask = nullptr;
};

} // namespace mc_control

using Controller = mc_control::TestManipulabilityEllipsoidTaskController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestManipulabilityEllipsoidTaskController",
                              Controller(rm, dt, Backend::Tasks),
                              "TestManipulabilityEllipsoidTaskController_TVM",
                              Controller(rm, dt, Backend::TVM))
