// Controller
#include "kimm_action_manager/controller/controller.hpp"

//Mujoco MSG Header
#include "mujoco_ros_msgs/JointSet.h"
#include "mujoco_ros_msgs/SensorState.h"

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

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

//HuskyFrankaAction
#include <kimm_action_manager/servers/joint_posture_action_server.hpp>
#include <kimm_action_manager/servers/se3_action_server.hpp>
#include <kimm_action_manager/servers/se3_array_action_server.hpp>
#include <kimm_action_manager/servers/gripper_action_server.hpp>
#include <kimm_action_manager/servers/move_action_server.hpp>

// Tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace Eigen;

std::shared_ptr<RobotController::HuskyFrankaWrapper> ctrl_;

//pub
ros::Publisher joint_state_publisher_;
ros::Publisher husky_pose_publisher_;
ros::Publisher mujoco_command_pub_;
ros::Publisher robot_command_pub_;
ros::Publisher gui_joint_pub_;

//msg
mujoco_ros_msgs::JointSet robot_command_msg_;
sensor_msgs::JointState gui_joint_msg_;

//tf
tf::TransformBroadcaster* br_;

//Action Server
std::unique_ptr<JointPostureActionServer> joint_posture_action_server_;
std::unique_ptr<SE3ActionServer> se3_action_server_;
std::unique_ptr<SE3ArrayActionServer> se3_array_action_server_;
std::unique_ptr<GripperActionServer> gripper_action_server_;
std::unique_ptr<MoveActionServer> move_action_server_;

double mujoco_time_, time_;
bool gravity_ctrl_;
string group_name_;
double vel_left_, vel_right_;

void simCommandCallback(const std_msgs::StringConstPtr &msg);
void simTimeCallback(const std_msgs::Float32ConstPtr &msg);
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
void gravityCallback(const std_msgs::BoolConstPtr &msg);
void cmdCallback(const geometry_msgs::TwistConstPtr & msg);