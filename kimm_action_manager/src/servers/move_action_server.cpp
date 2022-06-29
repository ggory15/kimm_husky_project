#include <kimm_action_manager/servers/move_action_server.hpp>

MoveActionServer::MoveActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&MoveActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&MoveActionServer::preemptCallback, this));
    as_.start();

    string group_name;
    nh.getParam("/robot_group", group_name);   

    move_base_subscriber_ = nh.subscribe("/" + group_name + "/move_base/result", 1, &MoveActionServer::movebaseCallback, this);
    move_base_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/" + group_name + "/move_base_simple/goal", 100);   

    move_done_ = false;
}

void MoveActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();

    move_base_publisher_.publish(goal_->target_pose);
    start_time_ = ros::Time::now();

    control_running_ = true;  
}

void MoveActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool MoveActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  if (move_done_){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec() > 20.0){
    setAborted();
    return false;
  }

  return false;
}


void MoveActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void MoveActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void MoveActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}
void MoveActionServer::movebaseCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
    if (msg->status.status == 3)
      move_done_ = true;
    else
      move_done_ = false;
}