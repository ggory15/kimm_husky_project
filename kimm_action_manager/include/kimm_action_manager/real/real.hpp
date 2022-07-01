// Controller
#include "kimm_action_manager/controller/controller.hpp"

// gazebo-like controller interface
#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>

// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <realtime_tools/realtime_publisher.h>
#include "visualization_msgs/Marker.h"

//SYSTEM Header
#include <iostream>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/resource.h>

// Franka
#include <franka/robot_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/trigger_rate.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>

//HuskyFrankaAction
#include <kimm_action_manager/servers/joint_posture_action_server.hpp>
#include <kimm_action_manager/servers/se3_action_server.hpp>
#include <kimm_action_manager/servers/se3_array_action_server.hpp>
#include <kimm_action_manager/servers/gripper_action_server.hpp>
#include <kimm_action_manager/servers/move_action_server.hpp>
#include <kimm_action_manager/servers/qr_action_server.hpp>

// Tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace Eigen;

std::shared_ptr<RobotController::HuskyFrankaWrapper> ctrl_;

namespace kimm_action_manager {

class BasicHuskyFrankaController : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time& time) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void stopping(const ros::Time& /*time*/);

    void asyncCalculationProc(); 

    void odomCallback(const nav_msgs::OdometryConstPtr &msg);
    void huskystateCallback(const sensor_msgs::JointStateConstPtr &msg);
    void gravityCallback(const std_msgs::BoolConstPtr &msg);
    double lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency){
        double gain = sample_time / (sample_time + (1.0 / (2.0 * M_PI * cutoff_frequency)));
        return gain * y + (1 - gain) * y_last;
    }

 private: 
    ros::Publisher gui_joint_pub_;
    ros::Publisher husky_odom_pub_;
    realtime_tools::RealtimePublisher<geometry_msgs::Twist> husky_ctrl_pub_;

    tf::TransformBroadcaster* br_;
    tf::TransformListener listener_;

    ros::Subscriber odom_subs_, husky_state_subs_, gravity_ctrl_subs_;

    sensor_msgs::JointState husky_state_msg_, gui_joint_msg_;
    nav_msgs::Odometry odom_msg_;

    // thread
    std::mutex calculation_mutex_;
    std::thread async_calculation_thread_, mode_change_thread_;
    bool is_calculated_{false}, quit_all_proc_{false}, gravity_ctrl_{true}, isslam_{false};

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
    std::string group_name_;

    actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_ac_{"franka_gripper/move", true};
    actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_grasp_ac_{"franka_gripper/grasp", true};
    franka_gripper::GraspGoal goal;
    franka_hw::TriggerRate print_rate_trigger_{100}; 
    franka_hw::TriggerRate husky_base_control_trigger_{100};
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);

    std::unique_ptr<JointPostureActionServer> joint_posture_action_server_;
    std::unique_ptr<SE3ActionServer> se3_action_server_;
    std::unique_ptr<SE3ArrayActionServer> se3_array_action_server_;
    std::unique_ptr<MoveActionServer> move_action_server_;
    std::unique_ptr<QRActionServer> qr_action_server_;

    // Variables
    const double delta_tau_max_{30.0};
    Vector7d dq_filtered_, franka_torque_, franka_q_;
    Vector6d f_filtered_, f_;
    Eigen::VectorXd franka_qacc_, robot_nle_, robot_g_, husky_qvel_, husky_qacc_, husky_qvel_prev_;
    Vector3d odom_lpf_, odom_dot_lpf_, odom_lpf_prev_, odom_dot_lpf_prev_, slam_lpf_prev_, slam_lpf_;
    Vector2d husky_cmd_, wheel_vel_;
    MatrixXd robot_mass_, robot_J_, robot_tau_;
    double time_;
    bool isgrasp_{false};


    std::shared_ptr<RobotController::HuskyFrankaWrapper> ctrl_;
 
};

}  // namespace kimm_franka_controllers


//Action Server


// double mujoco_time_, time_;
// bool gravity_ctrl_;
// string group_name_;
// double vel_left_, vel_right_;

// void simCommandCallback(const std_msgs::StringConstPtr &msg);
// void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
// void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
// void gravityCallback(const std_msgs::BoolConstPtr &msg);
// void cmdCallback(const geometry_msgs::TwistConstPtr & msg);