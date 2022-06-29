#ifndef __husky_franka_ctrl__
#define __husky_franka_ctrl__

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

//ROS Header
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int16.h"
#include "tf/transform_datatypes.h"

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// for hqp controller
#include <kimm_hqp_controller/robot/robot_wrapper.hpp>
#include <kimm_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp>
#include <kimm_hqp_controller/tasks/task_se3_equality.hpp>
#include <kimm_hqp_controller/tasks/task_joint_posture.hpp>
#include <kimm_hqp_controller/tasks/task_joint_bound.hpp>
#include <kimm_hqp_controller/tasks/task_mobile_base.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_euclidian.hpp>
#include <kimm_hqp_controller/trajectory/trajectory_se3.hpp>
#include <kimm_hqp_controller/solver/solver_HQP_factory.hxx>
#include <kimm_hqp_controller/solver/util.hpp>
#include <kimm_hqp_controller/math/util.hpp>

using namespace std;
using namespace Eigen;
using namespace pinocchio;

typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;
typedef struct Franka_State {
    Vector7d q;
    Vector7d v;
    Vector7d q_des;
    Vector7d v_des;
    Vector7d q_ref;
    Vector7d v_ref;
    Vector7d torque;
    Vector7d acc;
    SE3 H_ee_ref;
    SE3 H_ee_init;
    SE3 H_ee;
    Vector3d ee_offset;
    std::vector<SE3> H_ee_ref_array;
    std::vector<double> duration_array;
    int array_cnt;
    Matrix7d M;
    Vector7d G;
    double duration;
    bool isrelative;
    bool iswholebody;
    bool isgrasp;
    std::vector<bool> isrelative_array;
    std::vector<bool> iswholebody_array;

} franka_state;

typedef struct Husky_State {
    Vector5d q;
    Vector5d v;
    Vector2d wheel_vel;
    double duration;
} husky_state;
typedef struct State {   
    franka_state franka;
    husky_state husky;
    VectorXd q; 
    VectorXd v; 
    VectorXd dv;
    VectorXd torque;
    
    double time;
    double duration;
    bool reset;
} state;   

namespace RobotController{
    class HuskyFrankaWrapper{
        public: 
            HuskyFrankaWrapper(const std::string & robot_node, const bool & issimulation, ros::NodeHandle & node);
            ~HuskyFrankaWrapper(){};

            void initialize();
            void franka_update(const Vector7d& q, const Vector7d& qdot); 
            void husky_update(const Vector3d& base_pos, const Vector3d& base_vel, const Vector2d& wheel_pos, const Vector2d& wheel_vel);
            void compute_all_terms();
            
            void compute_default_ctrl(ros::Time time);
            void init_joint_posture_ctrl(ros::Time time);
            void compute_joint_posture_ctrl(ros::Time time);
            void init_se3_ctrl(ros::Time time);
            void compute_se3_ctrl(ros::Time time);
            void init_se3_array_ctrl(ros::Time time);
            void compute_se3_array_ctrl(ros::Time time);

            State & state(){
                return state_;
            }
            bool simulation(){
                return issimulation_;
            }
            void computeAllTerms(){
                robot_->computeAllTerms(data_, state_.q, state_.v);
                state_.v.setZero(); 
            }

        private:
            bool issimulation_, mode_change_, franka_gripper_;
            std::string robot_node_;
            
            State state_;
            double time_;
            double stime_;
            int ctrl_mode_;
            int array_cnt_;
            int na_, nv_, nq_;
            bool isfinished_;

            std::shared_ptr<kimmhqp::robot::RobotWrapper> robot_;
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::shared_ptr<kimmhqp::InverseDynamicsFormulationAccForce> tsid_;           
            std::shared_ptr<kimmhqp::tasks::TaskJointPosture> postureTask_;
            std::shared_ptr<kimmhqp::tasks::TaskSE3Equality> eeTask_;
            std::shared_ptr<kimmhqp::tasks::TaskJointBounds> torqueBoundsTask_;
            std::shared_ptr<kimmhqp::tasks::TaskMobileEquality> mobileTask_, mobileTask2_;

            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianCubic> trajPosture_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianConstant> trajPosture_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectoryEuclidianTimeopt> trajPosture_Timeopt_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Cubic> trajEE_Cubic_, trajMobile_Cubic_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Constant> trajEE_Constant_, trajMobile_Constant_;
            std::shared_ptr<kimmhqp::trajectory::TrajectorySE3Timeopt> trajEE_Timeopt_, trajMobile_Timeopt_;            

            kimmhqp::trajectory::TrajectorySample sampleEE_, samplePosture_, sampleMobile_;
            kimmhqp::solver::SolverHQPBase * solver_;

            ros::NodeHandle n_node_;
    };
}
#endif


