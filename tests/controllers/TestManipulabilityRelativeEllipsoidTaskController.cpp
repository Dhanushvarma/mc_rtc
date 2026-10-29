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

/** The arms of JVRC-1 track the translation ellipsoid of the right gripper relative to the left gripper (bimanual
 * relative manipulability) in another posture of both arms */
struct MC_CONTROL_DLLAPI TestManipulabilityRelativeEllipsoidTaskController : public MCController
{
public:
  TestManipulabilityRelativeEllipsoidTaskController(mc_rbdyn::RobotModulePtr rm, double dt, Backend backend)
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

    ellipsoidTask = std::make_shared<mc_tasks::ManipulabilityEllipsoidTask>(robot().frame("RightGripper"),
                                                                            robot().frame("LeftGripper"));
    BOOST_CHECK_EQUAL(ellipsoidTask->size(), 3);
    BOOST_REQUIRE(ellipsoidTask->referenceFrame());
    // The measure joints are both arms, the left one first: the waist joints move both grippers together
    std::vector<std::string> arms = leftArm;
    arms.insert(arms.end(), rightArm.begin(), rightArm.end());
    BOOST_CHECK(ellipsoidTask->measureJoints() == arms);

    mc_rtc::log::success("Created TestManipulabilityRelativeEllipsoidTaskController");
  }

  bool run() override
  {
    bool ret = MCController::run();
    if(!ret) { mc_rtc::log::critical("Failed at iter: {}", nrIter); }
    BOOST_CHECK(ret);
    nrIter++;
    if(nrIter == 100)
    {
      // Target: the relative ellipsoid with the right arm moved forward and the left elbow bent further
      const auto & q = robot().mbc().q;
      auto jointQ = [&](const std::string & j) { return q[robot().jointIndexByName(j)][0]; };
      ellipsoidTask->reset();
      ellipsoidTask->targetFromPosture({{"R_SHOULDER_P", {jointQ("R_SHOULDER_P") - 0.4}},
                                        {"R_ELBOW_P", {jointQ("R_ELBOW_P") - 0.3}},
                                        {"L_SHOULDER_Y", {jointQ("L_SHOULDER_Y") - 0.3}},
                                        {"L_ELBOW_P", {jointQ("L_ELBOW_P") - 0.5}}});
      std::vector<std::string> arms = leftArm;
      arms.insert(arms.end(), rightArm.begin(), rightArm.end());
      ellipsoidTask->selectActiveJoints(solver(), arms);
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
      mc_rtc::log::info("[TestManipulabilityRelativeEllipsoidTaskController] Distance to the target {} -> {}", d0, d);
      BOOST_CHECK_LT(d, 0.1 * d0);
    }
    return ret;
  }

private:
  unsigned int nrIter = 0;
  double d0 = 0.;
  std::vector<std::string> leftArm = {"L_SHOULDER_P", "L_SHOULDER_R", "L_SHOULDER_Y", "L_ELBOW_P",
                                      "L_ELBOW_Y",    "L_WRIST_R",    "L_WRIST_Y"};
  std::vector<std::string> rightArm = {"R_SHOULDER_P", "R_SHOULDER_R", "R_SHOULDER_Y", "R_ELBOW_P",
                                       "R_ELBOW_Y",    "R_WRIST_R",    "R_WRIST_Y"};
  std::shared_ptr<mc_tasks::ManipulabilityEllipsoidTask> ellipsoidTask = nullptr;
};

} // namespace mc_control

using Controller = mc_control::TestManipulabilityRelativeEllipsoidTaskController;
using Backend = mc_control::MCController::Backend;
MULTI_CONTROLLERS_CONSTRUCTOR("TestManipulabilityRelativeEllipsoidTaskController",
                              Controller(rm, dt, Backend::Tasks),
                              "TestManipulabilityRelativeEllipsoidTaskController_TVM",
                              Controller(rm, dt, Backend::TVM))
