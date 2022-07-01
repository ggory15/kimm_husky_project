#pragma once

#include <kimm_action_manager/servers/action_server_base.hpp>
#include <kimm_action_manager/QRAction.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"

#include <fstream>

using namespace pinocchio;
using namespace Eigen;

class QRActionServer : public ActionServerBase
{
  actionlib::SimpleActionServer<kimm_action_manager::QRAction> as_;

  kimm_action_manager::QRFeedback feedback_;
  kimm_action_manager::QRResult result_;
  kimm_action_manager::QRGoalConstPtr goal_;
  bool isrelative_, iswholebody_;

  geometry_msgs::Pose qr_msg_;
  SE3 qr_tf_;

  void goalCallback() override;
  void preemptCallback() override;

public:
  QRActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper>  &mu);

  bool compute(ros::Time time) override;
  void signalAbort(bool is_aborted) override;

protected:
  void setSucceeded() override;
  void setAborted() override;
  bool computeArm(ros::Time time);
};