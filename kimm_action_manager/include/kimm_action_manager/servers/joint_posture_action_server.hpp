#pragma once

#include <kimm_action_manager/servers/action_server_base.hpp>
#include <kimm_action_manager/JointPostureAction.h>

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class JointPostureActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<kimm_action_manager::JointPostureAction> as_;

  kimm_action_manager::JointPostureFeedback feedback_;
  kimm_action_manager::JointPostureResult result_;
  kimm_action_manager::JointPostureGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};