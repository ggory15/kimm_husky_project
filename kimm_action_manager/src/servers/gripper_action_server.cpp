#include <kimm_action_manager/servers/gripper_action_server.hpp>

GripperActionServer::GripperActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&GripperActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&GripperActionServer::preemptCallback, this));
    as_.start();
}

void GripperActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mu_->state().franka.isgrasp = goal_->grasp;
    
    start_time_ = ros::Time::now();
    mu_->init_joint_posture_ctrl(start_time_);
    control_running_ = true;  
}

void GripperActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool GripperActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if (ctime.toSec() - start_time_.toSec() > 0.1){
    setSucceeded();
    return true;
  }

  return false;
}


void GripperActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void GripperActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void GripperActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}