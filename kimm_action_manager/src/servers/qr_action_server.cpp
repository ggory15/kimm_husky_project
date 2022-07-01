#include <kimm_action_manager/servers/qr_action_server.hpp>
#include <pinocchio/fwd.hpp>

using namespace pinocchio;

QRActionServer::QRActionServer(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper> &mu)
: ActionServerBase(name,nh,mu), as_(nh,name,false)
{
    as_.registerGoalCallback(boost::bind(&QRActionServer::goalCallback, this));
    as_.registerPreemptCallback(boost::bind(&QRActionServer::preemptCallback, this));
    as_.start();
    isrelative_ = true;
    iswholebody_ = false;
}

void QRActionServer::goalCallback()
{
    feedback_header_stamp_ = 0;
    goal_ = as_.acceptNewGoal();
    
    mode_change_ = false;
    mu_->state().franka.isrelative = false;
    mu_->state().franka.iswholebody = true;

    Eigen::Matrix3d rot_offset; //[0 -1 0; 1 0 0; 0 0 1]
    rot_offset.setZero();
    rot_offset.col(0) << cos(M_PI/4.0), sin(M_PI/4.0), 0;
    rot_offset.col(1) << cos(M_PI/4.0), -sin(M_PI/4.0), 0;
    rot_offset.col(2) << 0, 0, -1;
    
    SE3 target_tf;
    qr_tf_ = SE3(Eigen::Quaterniond(goal_->qr_pose.orientation.w, goal_->qr_pose.orientation.x, goal_->qr_pose.orientation.y, goal_->qr_pose.orientation.z).toRotationMatrix(),
                 Eigen::Vector3d(goal_->qr_pose.position.x, goal_->qr_pose.position.y, goal_->qr_pose.position.z));
    target_tf = SE3(Eigen::Quaterniond(goal_->target_pose.orientation.w, goal_->target_pose.orientation.x, goal_->target_pose.orientation.y, goal_->target_pose.orientation.z).toRotationMatrix(),
                    Eigen::Vector3d(goal_->target_pose.position.x, goal_->target_pose.position.y, goal_->target_pose.position.z));             
    
    SE3 oMi_ref = qr_tf_ * target_tf;
    oMi_ref.rotation() = oMi_ref.rotation() * rot_offset;
    mu_->state().franka.H_ee_ref = oMi_ref;
    mu_->state().franka.duration = goal_->duration;

    start_time_ = ros::Time::now();
    mu_->init_se3_ctrl(start_time_);
    control_running_ = true;  
}

void QRActionServer::preemptCallback()
{
  ROS_INFO("[%s] Preempted", action_name_.c_str());
  as_.setPreempted();
  control_running_ = false;
}

bool QRActionServer::compute(ros::Time ctime)
{
  if (!control_running_)
    return false;

  if (!as_.isActive())
      return false; 
  
  mu_->compute_se3_ctrl(ctime);
  
  if (ctime.toSec() - start_time_.toSec() > goal_->duration + 1.0 && (mu_->state().franka.H_ee.translation()-mu_->state().franka.H_ee_ref.translation()).norm() < 5e-3){
    setSucceeded();
    return true;
  }

  if (ctime.toSec() - start_time_.toSec()  > goal_->duration+ 2.0){
    setAborted();
    return false;
  }

  return false;
}


void QRActionServer::signalAbort(bool is_aborted)
{
  setAborted();  
}

void QRActionServer::setSucceeded()
{
  as_.setSucceeded(result_);
  control_running_ = false;
}
void QRActionServer::setAborted()
{
  as_.setAborted();
  control_running_ = false;
}