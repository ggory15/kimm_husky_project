#pragma once

#include <kimm_action_manager/servers/action_server_base.hpp>
#include <kimm_action_manager/GripperAction.h>

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class GripperActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<kimm_action_manager::GripperAction> as_;

  kimm_action_manager::GripperFeedback feedback_;
  kimm_action_manager::GripperResult result_;
  kimm_action_manager::GripperGoalConstPtr goal_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  GripperActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};