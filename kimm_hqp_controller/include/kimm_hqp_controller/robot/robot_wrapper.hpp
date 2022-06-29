#ifndef __robots_robot_wrapper_hpp__
#define __robots_robot_wrapper_hpp__

#include "kimm_hqp_controller/math/fwd.hpp"
#include "kimm_hqp_controller/robot/fwd.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/fwd.hpp>

#include <string>
#include <vector>

namespace kimmhqp{
    namespace robot{
        class RobotWrapper{
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                typedef pinocchio::Model Model;
                typedef pinocchio::Data Data;
                typedef pinocchio::Motion Motion;
                typedef pinocchio::Frame Frame;
                typedef pinocchio::SE3 SE3;
                typedef math::Vector  Vector;
                typedef math::Vector3 Vector3;
                typedef math::Vector6 Vector6;
                typedef math::Matrix Matrix;
                typedef math::Matrix3x Matrix3x;
                typedef math::Matrix6d Matrix6d;
                typedef math::RefVector RefVector;
                typedef math::ConstRefVector ConstRefVector;
                

                RobotWrapper(const std::string & filename, const std::vector<std::string> & package_dirs, bool mobile=false, bool verbose=false);
                ~RobotWrapper(){};
                
                virtual int nq() const;
                virtual int nv() const;
                virtual int na() const;

                const Model & model() const;
                Model & model();

                void computeAllTerms(Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v);
                
                const Eigen::Vector3d & com(const Data & data) const;

                Eigen::VectorXd nonLinearEffects(const Data & data) ;

                const SE3 & position(const Data & data, const Model::JointIndex index) const;

                const Eigen::MatrixXd & mass(const Data & data);

                const Eigen::MatrixXd & mass_inverse(const Data & data);

                const Eigen::MatrixXd & coriolis(const Data & data);

                const Motion & velocity(const Data & data, const Model::JointIndex index) const;

                const Motion & acceleration(const Data & data, const Model::JointIndex index) const;

                void jacobianWorld(const Data & data, const Model::JointIndex index, Data::Matrix6x & J);

                SE3 framePosition(const Data & data, const Model::FrameIndex index) const;

                void framePosition(const Data & data, const Model::FrameIndex index, SE3 & framePosition) const;

                Motion frameVelocity(const Data & data, const Model::FrameIndex index) const;

                void frameVelocity(const Data & data, const Model::FrameIndex index, Motion & frameVelocity) const;

                Motion frameAcceleration(const Data & data, const Model::FrameIndex index) const;

                void frameAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;

                Motion frameClassicAcceleration(const Data & data, const Model::FrameIndex index) const;

                void frameClassicAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const;

                void frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J)  ;

                const bool & ismobile() const {
                    return mobile_flag_;
                }

                void setMobileParam(double r, double b, double d){
                    r_ = r;
                    b_ = b;
                    d_ = d;
                    c_ = r_ / (2.0 * b_);
                }

                SE3 getMobilePosition(const Data & data, unsigned int q_index, std::string name = "panda_joint1"){
                    Eigen::Matrix3d rot_j1;
                    rot_j1.setZero();
                    rot_j1(0, 0) = cos(m_q(q_index));
                    rot_j1(0, 1) = sin(m_q(q_index));
                    rot_j1(1, 0) = -sin(m_q(q_index));
                    rot_j1(1, 1) = cos(m_q(q_index));
                    rot_j1(2, 2) = 1.0;

                    SE3 oMi = this->position(data, m_model.getJointId(name));
                    oMi.rotation() = rot_j1 * oMi.rotation();
                    //oMi.translation() = rot_j1 * oMi.translation();
                    return oMi;                     
                }
                void getMobilePosition(const Data & data, unsigned int q_index, const Model::FrameIndex index, SE3 & framePosition) const {
                    Eigen::Matrix3d rot_j1;
                    rot_j1.setZero();
                    rot_j1(0, 0) = cos(m_q(q_index));
                    rot_j1(0, 1) = sin(m_q(q_index));
                    rot_j1(1, 0) = -sin(m_q(q_index));
                    rot_j1(1, 1) = cos(m_q(q_index));
                    rot_j1(2, 2) = 1.0;

                    this->framePosition(data, index, framePosition);
                    framePosition.rotation() = rot_j1 * framePosition.rotation();
               
                }
                void getMobileVelocity(const Data & data, unsigned int q_index, const Model::FrameIndex index, Motion & v_frame) const {
                    Eigen::Matrix3d rot_j1;
                    rot_j1.setZero();
                    rot_j1(0, 0) = cos(m_q(q_index));
                    rot_j1(0, 1) = sin(m_q(q_index));
                    rot_j1(1, 0) = -sin(m_q(q_index));
                    rot_j1(1, 1) = cos(m_q(q_index));
                    rot_j1(2, 2) = 1.0;

                    this->frameVelocity(data, index, v_frame);
                   // v_frame.linear() = rot_j1 * v_frame.linear();
                    //v_frame.angular() = rot_j1 * v_frame.angular();
                }

                void getMobileClassicAcceleration(const Data & data, unsigned int q_index, const Model::FrameIndex index, Motion & frameAcceleration) const{
                    Eigen::Matrix3d rot_j1;
                    rot_j1.setZero();
                    rot_j1(0, 0) = cos(m_q(q_index));
                    rot_j1(0, 1) = sin(m_q(q_index));
                    rot_j1(1, 0) = -sin(m_q(q_index));
                    rot_j1(1, 1) = cos(m_q(q_index));
                    rot_j1(2, 2) = 1.0;

                    this->frameClassicAcceleration(data, index, frameAcceleration);
                    //frameAcceleration.linear() = rot_j1 * frameAcceleration.linear();
                    //frameAcceleration.angular() = rot_j1 * frameAcceleration.angular();
                }

                void getMobileJacobianLocal(Data & data, unsigned int q_index, const Model::FrameIndex index, Data::Matrix6x & J) {
                    Eigen::MatrixXd rot_j1(6, 6);
                    rot_j1.setZero();
                    rot_j1(0, 0) = cos(m_q(q_index));
                    rot_j1(0, 1) = sin(m_q(q_index));
                    rot_j1(1, 0) = -sin(m_q(q_index));
                    rot_j1(1, 1) = cos(m_q(q_index));
                    rot_j1(2, 2) = 1.0;
                    rot_j1.bottomRightCorner(3,3) = rot_j1.topLeftCorner(3, 3);
                    rot_j1.topLeftCorner(3, 3).setIdentity();

                    this->frameJacobianLocal(data, index, J);
                    //J = rot_j1 * J;
                }

            protected:
                bool mobile_flag_;
                Model m_model;
                std::string m_model_filename;
                bool m_verbose;
                int m_na;
                Eigen::MatrixXd m_M, m_Minv, m_C;
                Eigen::MatrixXd m_S, m_S_dot;
                Matrix6d m_Rot;
                double r_, b_, d_, c_; 
                Eigen::VectorXd m_q;
 
                
        };
    }
}


#endif
