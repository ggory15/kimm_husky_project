#include <kimm_action_manager/real/real.hpp>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <pinocchio/parsers/urdf.hpp>

namespace kimm_action_manager
{
    bool BasicHuskyFrankaController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
    {
        node_handle.getParam("/robot_group", group_name_);

        // husky
        husky_ctrl_pub_.init(node_handle, "/" + group_name_ + "/husky/cmd_vel", 4);
        husky_state_msg_.position.resize(4);
        husky_state_msg_.velocity.resize(4);
        husky_state_subs_ = node_handle.subscribe( "/" + group_name_ + "/husky/joint_states", 1, &BasicHuskyFrankaController::huskystateCallback, this);
        odom_subs_ = node_handle.subscribe( "/" + group_name_ + "/husky/odometry/filtered", 1, &BasicHuskyFrankaController::odomCallback, this);
        
        // franka
        std::vector<std::string> joint_names;
        std::string arm_id;
        ROS_WARN(
            "ForceExampleController: Make sure your robot's endeffector is in contact "
            "with a horizontal surface before starting the controller!");
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
            return false;
        }
        ROS_WARN_STREAM(arm_id);

        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
            ROS_ERROR(
                "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
                "controller init!");
            return false;
        }

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "ForceExampleController: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
            return false;
        }
        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                state_interface->getHandle("ns0_panda_robot"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                "ForceExampleController: Exception getting state handle from interface: " << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
            return false;
            }
        }
        
        // gripper
        gripper_ac_.waitForServer();
        gripper_grasp_ac_.waitForServer();

        ctrl_ = std::make_shared<RobotController::HuskyFrankaWrapper>(group_name_, true, node_handle);
        ctrl_->initialize();

        gravity_ctrl_subs_ = node_handle.subscribe("kimm_action_manager/gravity_ctrl", 1, &BasicHuskyFrankaController::gravityCallback, this);
        gravity_ctrl_ = true;

        // action_server
        joint_posture_action_server_ = std::make_unique<JointPostureActionServer>("kimm_action_manager/joint_posture_control", node_handle, ctrl_);
        se3_action_server_ = std::make_unique<SE3ActionServer>("kimm_action_manager/se3_control", node_handle, ctrl_);
        se3_array_action_server_ = std::make_unique<SE3ArrayActionServer>("kimm_action_manager/se3_array_control", node_handle, ctrl_);
        move_action_server_ = std::make_unique<MoveActionServer>("kimm_action_manager/move_control", node_handle, ctrl_);
        
        // gui_pub
        string model_path, urdf_name;
        node_handle.getParam("/" + group_name_ +"/gui_urdf_path", model_path);    
        node_handle.getParam("/" + group_name_ +"/gui_urdf_name", urdf_name);  
        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;
 
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
        gui_joint_pub_ = node_handle.advertise<sensor_msgs::JointState>("/" + group_name_ + "/joint_states", 100);

        return true;
    }

    void BasicHuskyFrankaController::starting(const ros::Time& time) {
        dq_filtered_.setZero();

        odom_lpf_prev_(0) = odom_msg_.pose.pose.position.x;
        odom_lpf_prev_(1) = odom_msg_.pose.pose.position.y;
        odom_lpf_prev_(2) = odom_msg_.pose.pose.orientation.z;

        odom_dot_lpf_prev_(0) = odom_msg_.twist.twist.linear.x;
        odom_dot_lpf_prev_(1) = odom_msg_.twist.twist.linear.y;
        odom_dot_lpf_prev_(2) = odom_msg_.twist.twist.angular.z;  

        f_filtered_.setZero();
        
        br_ = new tf::TransformBroadcaster(); 
    }
    void BasicHuskyFrankaController::stopping(const ros::Time& time){
        ROS_INFO("Robot Controller::stopping");
    }
    void BasicHuskyFrankaController::update(const ros::Time& time, const ros::Duration& period) {
        franka::RobotState robot_state = state_handle_->getRobotState();

        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        std::array<double, 7> gravity_array = model_handle_->getGravity();
        std::array<double, 49> massmatrix_array = model_handle_->getMass();
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
        robot_tau_ = tau_J_d;

        Eigen::Map<Vector7d> gravity(gravity_array.data());
        robot_g_ = gravity;
        Eigen::Map<Matrix7d> mass_matrix(massmatrix_array.data());
        robot_mass_ = mass_matrix;
        Eigen::Map<Vector7d> non_linear(coriolis_array.data());
        robot_nle_ = non_linear;
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
        robot_J_ = jacobian;
        Eigen::Map<Vector7d> franka_q(robot_state.q.data());
        Eigen::Map<Vector7d> franka_dq(robot_state.dq.data());
        franka_q_ = franka_q;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> force_franka(robot_state.O_F_ext_hat_K.data());
        f_ = force_franka;

        // Filtering
        double cutoff = 20.0; // Hz //20
        double RC = 1.0 / (cutoff * 2.0 * M_PI);
        double dt = 0.001;
        double alpha = dt / (RC + dt);
        
        dq_filtered_ = alpha * franka_dq + (1 - alpha) * dq_filtered_;
        f_filtered_ = alpha * f_ + (1 - alpha) * f_filtered_;

        odom_lpf_(0) = this->lowpassFilter(0.001, odom_msg_.pose.pose.position.x, odom_lpf_prev_(0), 100);
        odom_lpf_(1) = this->lowpassFilter(0.001, odom_msg_.pose.pose.position.y, odom_lpf_prev_(1), 100);
        odom_lpf_(2) = this->lowpassFilter(0.001, odom_msg_.pose.pose.orientation.z, odom_lpf_prev_(2), 100);
        
        odom_dot_lpf_(0) = this->lowpassFilter(0.001, odom_msg_.twist.twist.linear.x, odom_dot_lpf_prev_(0), 100);
        odom_dot_lpf_(1) = this->lowpassFilter(0.001, odom_msg_.twist.twist.linear.y, odom_dot_lpf_prev_(1), 100);
        odom_dot_lpf_(2) = this->lowpassFilter(0.001, odom_msg_.twist.twist.angular.z, odom_dot_lpf_prev_(2), 100);

        odom_lpf_prev_ = odom_lpf_;
        odom_dot_lpf_prev_ = odom_dot_lpf_;

        tf::Transform transform;
        transform.setOrigin( tf::Vector3(odom_lpf_(0), odom_lpf_(1), 0.0 ));
        
        tf::Quaternion quat;
        quat.setRPY(0, 0, odom_lpf_(2));
        transform.setRotation(quat);
        br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(), group_name_ + "_odom", group_name_ + "_rviz_base_link"));

        wheel_vel_(0) = husky_state_msg_.velocity[1]; // left vel
        wheel_vel_(1) = husky_state_msg_.velocity[0]; // right vel (not use.)
        
        // HQP update thread    
        if (calculation_mutex_.try_lock())
        {
            calculation_mutex_.unlock();
            if (async_calculation_thread_.joinable())
            async_calculation_thread_.join();
            async_calculation_thread_ = std::thread(&BasicHuskyFrankaController::asyncCalculationProc, this);
        }

        ros::Rate r(30000);
        for (int i = 0; i < 9; i++)
        {
            r.sleep();
            if (calculation_mutex_.try_lock())
            {
            calculation_mutex_.unlock();
            if (async_calculation_thread_.joinable())
                async_calculation_thread_.join();
            break;
            }
        }       

        ctrl_->compute_all_terms();
        joint_posture_action_server_->compute(ros::Time::now());
        se3_action_server_->compute(ros::Time::now());
        se3_array_action_server_->compute(ros::Time::now());
        move_action_server_->compute(ros::Time::now());

        if (!joint_posture_action_server_->isrunning() &&
            !se3_action_server_->isrunning() &&
            !se3_array_action_server_->isrunning()
            ){
            if (!gravity_ctrl_)
                ctrl_->compute_default_ctrl(ros::Time::now());
            else{
                ctrl_->state().franka.acc.setZero();
                ctrl_->state().husky.wheel_vel.setZero();
            }
        }
        
        // // Customizing
        robot_mass_(4, 4) *= 6.0;
        robot_mass_(5, 5) *= 6.0;
        robot_mass_(6, 6) *= 10.0;
        
        franka_torque_ = robot_mass_ * ctrl_->state().franka.acc + robot_nle_;

        MatrixXd Kd(7, 7);
        Kd.setIdentity();
        Kd = 2.0 * sqrt(5.0) * Kd;
        Kd(5, 5) = 0.2;
        Kd(4, 4) = 0.2;
        Kd(6, 6) = 0.2; // this is practical term
        franka_torque_ -= Kd * dq_filtered_;  
        franka_torque_ << this->saturateTorqueRate(franka_torque_, robot_tau_);

        husky_qvel_ = ctrl_->state().husky.wheel_vel  * 0.01;//  + husky_qvel_prev_;
        double thes_vel = 5.0;
        if (husky_qvel_(0) > thes_vel)
            husky_qvel_(0) = thes_vel;
        else if (husky_qvel_(0) < -thes_vel)
            husky_qvel_(0) = -thes_vel;
        if (husky_qvel_(1) > thes_vel)
            husky_qvel_(1) = thes_vel;
        else if (husky_qvel_(1) < -thes_vel)
            husky_qvel_(1) = -thes_vel;  
        
        if (abs(husky_qvel_(0)) < 0.1)
            husky_qvel_(0) = 0.0;
        if (abs(husky_qvel_(1)) < 0.1)
            husky_qvel_(1) = 0.0;

            
        husky_cmd_(0) = 0.165 * (husky_qvel_(0) + husky_qvel_(1)) / 2.0;
        husky_cmd_(1) = (-husky_qvel_(0) + husky_qvel_(1)) * 0.165;
        husky_cmd_ *= 1.0;


        // if (print_rate_trigger_())
        // {
        //   ROS_INFO("--------------------------------------------------");
        //   ROS_INFO_STREAM("franka_torque_ :" << franka_torque_.transpose());
        //   ROS_INFO_STREAM("husky_cmd_ :" << husky_cmd_.transpose());
        // }
        
        // franka_torque_.setZero();
        // husky_cmd_.setZero();

        // to robot
        for (int i = 0; i < 7; i++)
            joint_handles_[i].setCommand(franka_torque_(i));

        if (husky_base_control_trigger_())
        {
            if (husky_ctrl_pub_.trylock())
            {
                husky_ctrl_pub_.msg_.linear.x = husky_cmd_(0);
                husky_ctrl_pub_.msg_.angular.z = husky_cmd_(1);
                husky_ctrl_pub_.unlockAndPublish();
            }
        }
    }

    void BasicHuskyFrankaController::huskystateCallback(const sensor_msgs::JointStateConstPtr &msg)
    {
        if (msg->name.size()==4)
        husky_state_msg_ = *msg;      
    }
    void BasicHuskyFrankaController::odomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_msg_ = *msg;
    }

    void BasicHuskyFrankaController::asyncCalculationProc(){
        calculation_mutex_.lock();

        ctrl_->husky_update(odom_lpf_, odom_dot_lpf_, Vector2d::Zero(), wheel_vel_);
        ctrl_->franka_update(franka_q_, dq_filtered_);

        gui_joint_msg_.header.stamp = ros::Time::now();
        
        gui_joint_msg_.position[0] = husky_state_msg_.position[0];
        gui_joint_msg_.position[1] = husky_state_msg_.position[1];
        gui_joint_msg_.position[2] = husky_state_msg_.position[0];
        gui_joint_msg_.position[3] = husky_state_msg_.position[1];

        for (int i=0; i< 7; i++){ 
            gui_joint_msg_.position[i+4] = franka_q_(i);
        }

        gui_joint_pub_.publish(gui_joint_msg_);

        calculation_mutex_.unlock();
    }
    Eigen::Matrix<double, 7, 1> BasicHuskyFrankaController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] =
                tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }
    void BasicHuskyFrankaController::gravityCallback(const std_msgs::BoolConstPtr &msg){
        ctrl_->state().reset = true;
        gravity_ctrl_ = msg->data;
    }


}; // namespace kimm_action_manager
PLUGINLIB_EXPORT_CLASS(kimm_action_manager::BasicHuskyFrankaController, controller_interface::ControllerBase)