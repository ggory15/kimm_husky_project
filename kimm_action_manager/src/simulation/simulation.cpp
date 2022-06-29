#include "kimm_action_manager/simulation/simulation.hpp"
#include <pinocchio/parsers/urdf.hpp>

using namespace std;
using namespace pinocchio;
using namespace Eigen;
using namespace RobotController;

int main(int argc, char **argv)
{  
    //Ros setting
    ros::init(argc, argv, "husky_franka_controller");
    ros::NodeHandle n_node;
    ros::Rate loop_rate(1000);

    // Robot Wapper
    n_node.getParam("/robot_group", group_name_);    
    ctrl_ = std::make_shared<RobotController::HuskyFrankaWrapper>(group_name_, true, n_node);
    ctrl_->initialize();

    // mujoco sub
    ros::Subscriber jointState = n_node.subscribe("mujoco_ros/mujoco_ros_interface/joint_states", 5, &JointStateCallback, ros::TransportHints().tcpNoDelay(true));    
    ros::Subscriber mujoco_command_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_command_sim2con", 5, &simCommandCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber mujoco_time_sub = n_node.subscribe("mujoco_ros/mujoco_ros_interface/sim_time", 1, &simTimeCallback, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber gravity_ctrl_sub = n_node.subscribe("kimm_action_manager/gravity_ctrl", 1, &gravityCallback, ros::TransportHints().tcpNoDelay(true));
    
    // mujoco pub
    mujoco_command_pub_ = n_node.advertise<std_msgs::String>("mujoco_ros/mujoco_ros_interface/sim_command_con2sim", 5);
    robot_command_pub_ = n_node.advertise<mujoco_ros_msgs::JointSet>("mujoco_ros/mujoco_ros_interface/joint_set", 5);

    // mujoco_msg
    robot_command_msg_.torque.resize(13); // gripper(2) + wheels(4) + robot (7)
    
    // movebase sub
    ros::Subscriber move_base_sub = n_node.subscribe("cmd_vel", 1, &cmdCallback, ros::TransportHints().tcpNoDelay(true));

    // action_server
    joint_posture_action_server_ = std::make_unique<JointPostureActionServer>("kimm_action_manager/joint_posture_control", n_node, ctrl_);
    se3_action_server_ = std::make_unique<SE3ActionServer>("kimm_action_manager/se3_control", n_node, ctrl_);
    se3_array_action_server_ = std::make_unique<SE3ArrayActionServer>("kimm_action_manager/se3_array_control", n_node, ctrl_);
    gripper_action_server_ = std::make_unique<GripperActionServer>("kimm_action_manager/gripper_control", n_node, ctrl_);
    move_action_server_ = std::make_unique<MoveActionServer>("kimm_action_manager/move_control", n_node, ctrl_);

    // gui_pub
    string model_path, urdf_name;
    n_node.getParam("/" + group_name_ +"/gui_urdf_path", model_path);    
    n_node.getParam("/" + group_name_ +"/gui_urdf_name", urdf_name);  
    vector<string> package_dirs;
    package_dirs.push_back(model_path);
    string urdfFileName = package_dirs[0] + urdf_name;
    ROS_INFO_STREAM(urdfFileName);
    pinocchio::Model gui_model;
    pinocchio::urdf::buildModel(urdfFileName, gui_model, false);       
    
    gui_joint_msg_.name.resize(gui_model.nq);
    gui_joint_msg_.position.resize(gui_model.nq);
    gui_joint_msg_.velocity.resize(gui_model.nq);
    gui_joint_msg_.effort.resize(gui_model.nq);

    for (int j=0; j<gui_model.nq; j++){
        gui_joint_msg_.name[j] = gui_model.names[j+1];
        gui_joint_msg_.position[j] = 0.0;
        gui_joint_msg_.velocity[j] = 0.0;
        gui_joint_msg_.effort[j] = 0.0;
    }
    gui_joint_pub_ = n_node.advertise<sensor_msgs::JointState>("/" + group_name_ + "/joint_states", 100);
    gravity_ctrl_ = true;

    //tf
    br_ = new tf::TransformBroadcaster();
    tf::TransformListener listener;

    while (ros::ok()){
        ctrl_->compute_all_terms();
        
        joint_posture_action_server_->compute(ros::Time::now());
        se3_action_server_->compute(ros::Time::now());
        se3_array_action_server_->compute(ros::Time::now());
        gripper_action_server_->compute(ros::Time::now());
        move_action_server_->compute(ros::Time::now());

        if (!joint_posture_action_server_->isrunning() &&
            !se3_action_server_->isrunning() &&
            !se3_array_action_server_->isrunning() &&
            !gripper_action_server_->isrunning() 
            ){
            if (!gravity_ctrl_)
                ctrl_->compute_default_ctrl(ros::Time::now());
            else
                ctrl_->state().franka.torque.setZero();
        }

        // publishJointState();
        // publishBodyPose();

        robot_command_msg_.MODE = 1;
        robot_command_msg_.header.stamp = ros::Time::now();
        robot_command_msg_.time = time_;
        robot_command_msg_.torque[0] = ctrl_->state().husky.wheel_vel(0) + vel_left_;
        robot_command_msg_.torque[2] = ctrl_->state().husky.wheel_vel(0) + vel_left_;
        robot_command_msg_.torque[1] = ctrl_->state().husky.wheel_vel(1) + vel_right_;
        robot_command_msg_.torque[3] = ctrl_->state().husky.wheel_vel(1) + vel_right_;

        for (int i=0; i<7; i++){
            robot_command_msg_.torque[i+4] = ctrl_->state().franka.G(i) + ctrl_->state().franka.torque(i);
        }
        if (ctrl_->state().franka.isgrasp){
            robot_command_msg_.torque[11] = -200.0;
            robot_command_msg_.torque[12] = -200.0;
        }
        else{
            robot_command_msg_.torque[11] = 100.0;
            robot_command_msg_.torque[12] = 100.0;
        }
        robot_command_pub_.publish(robot_command_msg_);
        gui_joint_msg_.header.stamp = ros::Time::now();
        
        
        gui_joint_pub_.publish(gui_joint_msg_);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void simCommandCallback(const std_msgs::StringConstPtr &msg){
    std::string buf;
    buf = msg->data;

    if (buf == "RESET")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "RESET";
        mujoco_command_pub_.publish(rst_msg_);
    }

    if (buf == "INIT")
    {
        std_msgs::String rst_msg_;
        rst_msg_.data = "INIT";
        mujoco_command_pub_.publish(rst_msg_);
        mujoco_time_ = 0.0;
    }
}

void simTimeCallback(const std_msgs::Float32ConstPtr &msg){
    mujoco_time_ = msg->data;
    time_ = mujoco_time_;
}

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    Vector3d q_mobile, v_mobile;
    Vector2d q_wheel, v_wheel;
    for (int i=0; i<2; i++){
        q_mobile(i) = msg->position[i];
        v_mobile(i) = msg->velocity[i];
        q_wheel(i) = msg->position[i+7];
        v_wheel(i) = msg->velocity[i+6];
    }
    gui_joint_msg_.position[0] = msg->position[7];
    gui_joint_msg_.position[1] = msg->position[8];
    gui_joint_msg_.position[2] = msg->position[7];
    gui_joint_msg_.position[3] = msg->position[8];
    q_mobile(2) = atan2(2.* (msg->position[5] * msg->position[4] + msg->position[6] * msg->position[3]), 1- 2.*(pow( msg->position[6], 2) + pow(msg->position[5], 2)));   
    v_mobile(2) = msg->velocity[5];
    ctrl_->husky_update(q_mobile, v_mobile, q_wheel, v_wheel);

    Vector7d q_arm, v_arm;
    for (int i=0; i< 7; i++){ 
        q_arm(i) = msg->position[i+11];
        v_arm(i) = msg->velocity[i+10];
        gui_joint_msg_.position[i+4] = ctrl_->state().q(i+5);
    }
    ctrl_->franka_update(q_arm, v_arm);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->position[0], msg->position[1], msg->position[2]) );
    tf::Quaternion q(msg->position[4], msg->position[5], msg->position[6], msg->position[3]);
    transform.setRotation(q);

    br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", group_name_ + "_rviz_base_link"));
}

void gravityCallback(const std_msgs::BoolConstPtr &msg){
    ctrl_->state().reset = true;
    gravity_ctrl_ = msg->data;
}

void cmdCallback(const geometry_msgs::TwistConstPtr & msg){
    vel_left_  = (msg->linear.x - msg->angular.z * 3.875 / 2.0) * 30.;
    vel_right_ = (msg->linear.x + msg->angular.z * 3.875 / 2.0) * 30.;
}