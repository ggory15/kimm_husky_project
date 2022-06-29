#include <kimm_action_manager/servers/joint_posture_action_server.hpp>

JointPostureActionServer::JointPostureActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
	as_.registerGoalCallback(boost::bind(&JointPostureActionServer::goalCallback, this));
	as_.registerPreemptCallback(boost::bind(&JointPostureActionServer::preemptCallback, this));
  as_.start();
}

void JointPostureActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mode_change_ = false;
    for (int i=0; i<7; i++)
        mu_->state().franka.q_ref(i) = goal_->target_joints.position[i];
    mu_->state().franka.duration = goal_->duration;
    
    start_time_ = ros::Time::now();
    mu_->init_joint_posture_ctrl(start_time_);
    control_running_ = true;  
}

void JointPostureActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool JointPostureActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->compute_joint_posture_ctrl(ctime);

  if (ctime.toSec() - start_time_.toSec() > goal_->duration && (mu_->state().franka.q_ref-mu_->state().franka.q).norm() < 1e-3){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration + 1.0){
    setAborted();
    return false;
  }

  return false;
}


void JointPostureActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void JointPostureActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void JointPostureActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}