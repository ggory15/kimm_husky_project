#pragma once

#include <kimm_action_manager/servers/action_server_base.hpp>
#include <kimm_action_manager/SE3Action.h>
#include "tf/transform_datatypes.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class SE3ActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<kimm_action_manager::SE3Action> as_;

  kimm_action_manager::SE3Feedback feedback_;
  kimm_action_manager::SE3Result result_;
  kimm_action_manager::SE3GoalConstPtr goal_;
  bool isrelative_, iswholebody_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  SE3ActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};