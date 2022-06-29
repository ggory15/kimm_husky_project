#pragma once

#include <kimm_action_manager/servers/action_server_base.hpp>
#include <kimm_action_manager/SE3ArrayAction.h>
#include "geometry_msgs/Pose.h"
#include <fstream>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include "pinocchio/fwd.hpp"

using namespace pinocchio;
using namespace Eigen;

class SE3ArrayActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<kimm_action_manager::SE3ArrayAction> as_;

  kimm_action_manager::SE3ArrayFeedback feedback_;
  kimm_action_manager::SE3ArrayResult result_;
  kimm_action_manager::SE3ArrayGoalConstPtr goal_;

  std::vector<bool> isrelative_, iswholebody_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  SE3ArrayActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};