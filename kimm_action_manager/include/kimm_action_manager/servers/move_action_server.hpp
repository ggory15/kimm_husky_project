#pragma once

#include <kimm_action_manager/servers/action_server_base.hpp>
#include <kimm_action_manager/MoveAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class MoveActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<kimm_action_manager::MoveAction> as_;

  kimm_action_manager::MoveFeedback feedback_;
  kimm_action_manager::MoveResult result_;
  kimm_action_manager::MoveGoalConstPtr goal_;

  ros::Publisher move_base_publisher_;
  ros::Subscriber move_base_subscriber_;

  bool move_done_;

  void goalCallback() override;
  void preemptCallback() override;
  void movebaseCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

public:
  MoveActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
};