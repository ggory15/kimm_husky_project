#include "kimm_action_manager/controller/controller.hpp"

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace kimmhqp;
using namespace kimmhqp::trajectory;
using namespace kimmhqp::math;
using namespace kimmhqp::tasks;
using namespace kimmhqp::solver;
using namespace kimmhqp::robot;


namespace RobotController{
    HuskyFrankaWrapper::HuskyFrankaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node)
    : robot_node_(robot_node), issimulation_(issimulation), n_node_(node)
    {
        time_ = 0.;
        mode_change_ = false;
        ctrl_mode_ = 999;
    }

    void HuskyFrankaWrapper::initialize(){
        string model_path, urdf_name;
        n_node_.getParam("/" + robot_node_ +"/urdf_path", model_path);    
        n_node_.getParam("/" + robot_node_ +"/urdf_name", urdf_name);  

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;

        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, true, false);
        model_ = robot_->model();
        na_ = robot_->na();
        nv_ = robot_->nv();
        nq_ = robot_->nq();

        // State
        state_.q.setZero(nq_);
        state_.v.setZero(nv_);
        state_.dv.setZero(nv_);
        state_.torque.setZero(na_);
        state_.franka.torque.setZero(na_-2);
        state_.husky.wheel_vel.setZero();
        state_.franka.isgrasp = false;
        state_.reset = true;

        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q, state_.v);
        data_ = tsid_->data();

        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
        Vector7d posture_gain;
        if (!issimulation_)
        	posture_gain << 200., 200., 200., 200., 200., 200., 200.;
        else
        	posture_gain << 4000., 4000., 4000., 4000., 4000., 4000., 4000.;

        posture_gain << 200., 200., 200., 200., 200., 200., 200.;
        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());

        Vector3d ee_offset(0.0, 0, 0.0);
        VectorXd ee_gain(6);
        ee_gain << 100., 100., 100., 800., 800., 800.;
        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7", ee_offset);
        eeTask_->Kp(ee_gain*Vector::Ones(6));
        eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());
        
        torqueBoundsTask_ = std::make_shared<TaskJointBounds>("task-torque-bounds", *robot_);
        Vector dq_max = 500000.0*Vector::Ones(na_);    
        dq_max(0) = 500.;
        dq_max(1) = 500.;
        Vector dq_min = -dq_max;
        torqueBoundsTask_->setJointBounds(dq_min, dq_max);

        mobileTask_ = std::make_shared<TaskMobileEquality>("task-mobile", *robot_, true);
        mobileTask_->Kp(50.0*Vector3d::Ones());    
        mobileTask_->Kd(2.5*mobileTask_->Kp().cwiseSqrt());

        mobileTask2_ = std::make_shared<TaskMobileEquality>("task-mobile2", *robot_, false);
        mobileTask2_->Kp(50.0*Vector3d::Ones());
        mobileTask2_->Kd(2.5*mobileTask2_->Kp().cwiseSqrt());

        // trajecotries
        sampleEE_.resize(12, 6);
        samplePosture_.resize(na_- 2);

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");
        
        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);

        Vector3d Maxvel_base = Vector3d::Ones()*1.0;
        Vector3d Maxacc_base = Vector3d::Ones()*1.0;
        trajMobile_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_mobile");
        trajMobile_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_mobile_constant");
        trajMobile_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_mobile_timeopt", Maxvel_base, Maxacc_base);

        // solver
        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog");           
    }

    void HuskyFrankaWrapper::franka_update(const Vector7d& q, const Vector7d& v){        
        state_.q.tail(7) = q;
        state_.v.tail(7) = v;
        state_.franka.q = q;
        state_.franka.v = v;
    }
    void HuskyFrankaWrapper::husky_update(const Vector3d& base_pos, const Vector3d& base_vel, const Vector2d& wheel_pos, const Vector2d& wheel_vel){     
        for (int i=0; i<3; i++){
            state_.q(i) = base_pos(i);
            state_.v(i) = base_vel(i);
        }
        for (int i=0; i<2; i++){
            state_.q(i+3) = wheel_pos(i);
            state_.v(i+3) = wheel_vel(i);
        }
        state_.husky.q = state_.q.head(5);
        state_.husky.v = state_.v.head(5);
    }
    void HuskyFrankaWrapper::compute_all_terms(){
        robot_->computeAllTerms(data_, state_.q, state_.v);

        state_.franka.G = robot_->nonLinearEffects(data_).tail(na_-2);
        state_.franka.M = robot_->mass(data_).bottomRightCorner(na_-2, na_-2);
        state_.franka.H_ee = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
    }

    void HuskyFrankaWrapper::compute_default_ctrl(ros::Time time){
        if (state_.reset){
            tsid_->removeTask("task-mobile");
            tsid_->removeTask("task-mobile2");
            tsid_->removeTask("task-se3");
            tsid_->removeTask("task-posture");
            tsid_->removeTask("task-torque-bounds");

            tsid_->addMotionTask(*postureTask_, 1e-5, 1);
            tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

            trajPosture_Cubic_->setInitSample(state_.franka.q);
            trajPosture_Cubic_->setDuration(0.1);
            trajPosture_Cubic_->setStartTime(time.toSec());
            trajPosture_Cubic_->setGoalSample(state_.franka.q);

            cout << "state reset " << endl;
            state_.reset = false;
        }
        trajPosture_Cubic_->setCurrentTime(time.toSec());
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);
    
        const HQPData & HQPData = tsid_->computeProblemData(time.toSec(), state_.q, state_.v);       
        state_.franka.torque = state_.franka.M * tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        state_.franka.acc = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        state_.husky.wheel_vel.setZero();
    }
    void HuskyFrankaWrapper::init_joint_posture_ctrl(ros::Time time){
        tsid_->removeTask("task-mobile");
        tsid_->removeTask("task-mobile2");
        tsid_->removeTask("task-se3");
        tsid_->removeTask("task-posture");
        tsid_->removeTask("task-torque-bounds");

        tsid_->addMotionTask(*postureTask_, 1e-5, 1);
        tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

        trajPosture_Cubic_->setInitSample(state_.franka.q);
        trajPosture_Cubic_->setDuration(state_.franka.duration);
        trajPosture_Cubic_->setStartTime(time.toSec());
        trajPosture_Cubic_->setGoalSample(state_.franka.q_ref);  
        ROS_WARN_STREAM(state_.franka.H_ee);
        
    }
    void HuskyFrankaWrapper::compute_joint_posture_ctrl(ros::Time time){
        trajPosture_Cubic_->setCurrentTime(time.toSec());
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);
    
        const HQPData & HQPData = tsid_->computeProblemData(time.toSec(), state_.q, state_.v);       
        state_.franka.torque = state_.franka.M * tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        state_.franka.acc = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);

        state_.reset = true;
    }

    void HuskyFrankaWrapper::init_se3_ctrl(ros::Time time){
        tsid_->removeTask("task-mobile");
        tsid_->removeTask("task-mobile2");
        tsid_->removeTask("task-se3");
        tsid_->removeTask("task-posture");
        tsid_->removeTask("task-torque-bounds");

        tsid_->addMotionTask(*postureTask_, 1e-2, 1);
        tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
        tsid_->addMotionTask(*eeTask_, 1.0, 0);

        state_.franka.q_ref = state_.franka.q;

        trajPosture_Cubic_->setInitSample(state_.franka.q);
        trajPosture_Cubic_->setDuration(0.1);
        trajPosture_Cubic_->setStartTime(time.toSec());
        trajPosture_Cubic_->setGoalSample(state_.franka.q_ref);   

        trajEE_Cubic_->setStartTime(time.toSec());
        trajEE_Cubic_->setDuration(state_.franka.duration);
        trajEE_Cubic_->setInitSample(state_.franka.H_ee);
        state_.franka.H_ee_init = state_.franka.H_ee;  

        if (state_.franka.isrelative){
            state_.franka.H_ee_ref.translation() = state_.franka.H_ee_ref.translation() + state_.franka.H_ee.translation();
            state_.franka.H_ee_ref.rotation() = state_.franka.H_ee.rotation() * state_.franka.H_ee_ref.rotation();
        }
        trajEE_Cubic_->setGoalSample(state_.franka.H_ee_ref);
        
        if (!state_.franka.iswholebody){
            tsid_->addMotionTask(*mobileTask2_, 1.0, 0);
            trajMobile_Cubic_->setStartTime(time.toSec());
            trajMobile_Cubic_->setDuration(0.001);
            trajMobile_Cubic_->setGoalSample(robot_->getMobilePosition(data_, 5));
            trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
        }
    }
    void HuskyFrankaWrapper::compute_se3_ctrl(ros::Time time){
        trajPosture_Cubic_->setCurrentTime(time.toSec());
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);

        trajEE_Cubic_->setCurrentTime(time.toSec());
        sampleEE_ = trajEE_Cubic_->computeNext();
        eeTask_->setReference(sampleEE_);
        
        if (!state_.franka.iswholebody){
            trajMobile_Cubic_->setCurrentTime(time.toSec());
            sampleMobile_ = trajMobile_Cubic_->computeNext();
            mobileTask2_->setReference(sampleMobile_);
        }
            
        const HQPData & HQPData = tsid_->computeProblemData(time.toSec(), state_.q, state_.v);       
        state_.franka.torque = state_.franka.M * tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        state_.franka.acc = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        state_.husky.wheel_vel = tsid_->getAccelerations(solver_->solve(HQPData)).head(2);
        if (!state_.franka.iswholebody){
            state_.husky.wheel_vel.setZero();
        }

        state_.reset = true;
    }

    void HuskyFrankaWrapper::init_se3_array_ctrl(ros::Time time){
        tsid_->removeTask("task-mobile");
        tsid_->removeTask("task-mobile2");
        tsid_->removeTask("task-se3");
        tsid_->removeTask("task-posture");
        tsid_->removeTask("task-torque-bounds");

        tsid_->addMotionTask(*postureTask_, 1e-2, 1);
        tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
        tsid_->addMotionTask(*eeTask_, 1.0, 0);

        state_.franka.q_ref = state_.franka.q;

        trajPosture_Cubic_->setInitSample(state_.franka.q);
        trajPosture_Cubic_->setDuration(0.1);
        trajPosture_Cubic_->setStartTime(time.toSec());
        trajPosture_Cubic_->setGoalSample(state_.franka.q_ref);   

        trajEE_Cubic_->setStartTime(time.toSec());
        trajEE_Cubic_->setDuration(state_.franka.duration_array[0]);
        trajEE_Cubic_->setInitSample(state_.franka.H_ee);
        state_.franka.H_ee_ref = state_.franka.H_ee_ref_array[0];
        state_.franka.H_ee_init = state_.franka.H_ee;  

        if (state_.franka.isrelative_array[0]){
            state_.franka.H_ee_ref.translation() = state_.franka.H_ee_ref_array[0].translation() + state_.franka.H_ee.translation();
            state_.franka.H_ee_ref.rotation() = state_.franka.H_ee.rotation() * state_.franka.H_ee_ref_array[0].rotation();
        }
        trajEE_Cubic_->setGoalSample(state_.franka.H_ee_ref);
        
        if (!state_.franka.iswholebody_array[0]){
            tsid_->addMotionTask(*mobileTask2_, 1.0, 0);
            trajMobile_Cubic_->setStartTime(time.toSec());
            trajMobile_Cubic_->setDuration(0.001);
            trajMobile_Cubic_->setGoalSample(robot_->getMobilePosition(data_, 5));
            trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
        }

        stime_ = time.toSec();
        state_.franka.array_cnt = 0;
    }
    void HuskyFrankaWrapper::compute_se3_array_ctrl(ros::Time time){
        trajPosture_Cubic_->setCurrentTime(time.toSec());
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);

        int array_size = state_.franka.duration_array.size();
        if ( (time.toSec() - stime_) >= state_.franka.duration_array[state_.franka.array_cnt] && state_.franka.array_cnt < array_size){     
            stime_ = time.toSec(); 
            state_.franka.array_cnt++;
            
            if (state_.franka.array_cnt < array_size){
                tsid_->removeTask("task-mobile");
                tsid_->removeTask("task-mobile2");

                state_.franka.q_ref = state_.franka.q;

                trajPosture_Cubic_->setInitSample(state_.franka.q);
                trajPosture_Cubic_->setDuration(0.1);
                trajPosture_Cubic_->setStartTime(time.toSec());
                trajPosture_Cubic_->setGoalSample(state_.franka.q_ref);   
                
                trajEE_Cubic_->setStartTime(stime_);
                trajEE_Cubic_->setDuration(state_.franka.duration_array[state_.franka.array_cnt]);
                trajEE_Cubic_->setInitSample(state_.franka.H_ee);     
                state_.franka.H_ee_ref = state_.franka.H_ee_ref_array[state_.franka.array_cnt];
                if (state_.franka.isrelative_array[state_.franka.array_cnt]){
                    Eigen::Vector3d pos_des;
                    Eigen::Matrix3d rot_des;
                    pos_des = state_.franka.H_ee_ref_array[0].translation();
                    rot_des = state_.franka.H_ee_ref_array[0].rotation();
                    for (int i=0; i<state_.franka.array_cnt; i++){
                        pos_des += state_.franka.H_ee_ref_array[i+1].translation();
                        rot_des *= state_.franka.H_ee_ref_array[i+1].rotation();
                    }
                    state_.franka.H_ee_ref.translation() = pos_des + state_.franka.H_ee_init.translation();
                    state_.franka.H_ee_ref.rotation() = state_.franka.H_ee_init.rotation() * rot_des;
                }
                trajEE_Cubic_->setGoalSample(state_.franka.H_ee_ref);    
                if (!state_.franka.iswholebody_array[state_.franka.array_cnt]){
                    trajMobile_Cubic_->setStartTime(time.toSec());
                    trajMobile_Cubic_->setDuration(0.001);
                    trajMobile_Cubic_->setGoalSample(robot_->getMobilePosition(data_, 5));
                    trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
                }
            }            
        }

        trajEE_Cubic_->setCurrentTime(time.toSec());
        sampleEE_ = trajEE_Cubic_->computeNext();
        eeTask_->setReference(sampleEE_);
        
        if (!state_.franka.iswholebody_array[state_.franka.array_cnt]){
            trajMobile_Cubic_->setCurrentTime(time.toSec());
            sampleMobile_ = trajMobile_Cubic_->computeNext();
            mobileTask2_->setReference(sampleMobile_);
        }
            
        const HQPData & HQPData = tsid_->computeProblemData(time.toSec(), state_.q, state_.v);       
        state_.franka.torque = state_.franka.M * tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);
        state_.husky.wheel_vel = tsid_->getAccelerations(solver_->solve(HQPData)).head(2);
        state_.franka.acc = tsid_->getAccelerations(solver_->solve(HQPData)).tail(na_-2);

        if (!state_.franka.iswholebody_array[state_.franka.array_cnt]){
            state_.husky.wheel_vel.setZero();
        }

        state_.reset = true;
    }




    

} //namespace