#pragma once

#include <kimm_action_manager/controller/controller.hpp>
#include <actionlib/server/simple_action_server.h>
#include <Eigen/Dense>
#include <map>

#define DEBUG_FILE(text) \
if(debug_file_.is_open()) \
{ \
  debug_file_ << text << std::endl;\
}

class ActionServerBase
{
  protected:
    int feedback_header_stamp_ {0}; 

    std::string action_name_;
    ros::Time start_time_;
    ros::NodeHandle & nh_; 
    bool control_running_ {false}; 
    bool mode_change_ {false};
    
    std::ofstream debug_file_;
    std::shared_ptr<RobotController::HuskyFrankaWrapper> mu_;
       
    ActionServerBase(std::string name, ros::NodeHandle &nh, std::shared_ptr<RobotController::HuskyFrankaWrapper> &mu) :
    action_name_(name), nh_(nh), mu_(mu)  {}

    virtual void goalCallback() = 0;
    virtual void preemptCallback() = 0;

  public:
    bool isrunning(){
        return control_running_;
    }
    virtual bool compute(ros::Time time) = 0;
    virtual void signalAbort(bool is_aborted)
    {
      setAborted();
    }

    virtual void openDebugFile(const std::string& prefix)
    {
      const auto now = std::chrono::system_clock::now();
      const auto now_time_t = std::chrono::system_clock::to_time_t(now);
      const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
      std::stringstream ss;
      ss << prefix << action_name_ << '_'
         << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %a %T")
         << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << ".txt";
        // std::cout << "[sss]: " << ss.str() << std::endl;
      debug_file_.open(ss.str());
    }

  protected:
    int iter_per_print_ {10};
    int print_count_ {0};
    
    void writeDebugInfos(const std::string &title, const std::string &context)
    {
      const auto now = std::chrono::system_clock::now();
      const auto now_time_t = std::chrono::system_clock::to_time_t(now);
      const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now.time_since_epoch()) % 1000;
      debug_file_ << "["
          << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %a %T")
          << '.' << std::setfill('0') << std::setw(3) << now_ms.count() << "]-";
      
      debug_file_ << "[" << title << "]: " << context << std::endl;
      debug_file_.precision(4);
      debug_file_ << std::setfill(' ');
    }
    virtual void setSucceeded() {};
    virtual void setAborted() {};
};
