#include "kimm_hqp_controller/robot/robot_wrapper.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

using namespace pinocchio;
using namespace std;

namespace kimmhqp{
    namespace robot{
        RobotWrapper::RobotWrapper(const std::string & filename, const std::vector<std::string> & , bool mobile, bool verbose)
        :m_verbose(verbose), mobile_flag_(mobile)
        {
           pinocchio::urdf::buildModel(filename, m_model, m_verbose);
           m_model_filename = filename;
           m_na = nv();
           if (mobile_flag_){
               m_na = m_model.nv-3;
               m_S.setZero(this->na() + 3, this->na());
               m_S_dot.setZero(this->na() + 3 , this->na()); 
               m_Rot.setZero();
               setMobileParam(0.165, 0.5, 0.05);
           }
        }

        const Model & RobotWrapper::model() const { return m_model; }
        Model & RobotWrapper::model() { return m_model; }

        int RobotWrapper::nq() const { return m_model.nq; }
        int RobotWrapper::nv() const { return m_model.nv; }
        int RobotWrapper::na() const { return m_na; }


        void RobotWrapper::computeAllTerms(Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v) 
        {
            m_q = q;
            pinocchio::computeAllTerms(m_model, data, q, v);
            data.M.triangularView<Eigen::StrictlyLower>()
                    = data.M.transpose().triangularView<Eigen::StrictlyLower>();
            // computeAllTerms does not compute the com acceleration, so we need to call centerOfMass
            // Check this line, calling with zero acceleration at the last phase compute the CoM acceleration.
            //      pinocchio::centerOfMass(m_model, data, q,v,false);
            pinocchio::updateFramePlacements(m_model, data);
            pinocchio::centerOfMass(m_model, data, q, v, Eigen::VectorXd::Zero(nv()));
            pinocchio::ccrba(m_model, data, q, v);

            if (mobile_flag_){                
                double theta = q(2);
               
                m_S.bottomRightCorner(this->na(), this->na()).setIdentity();
                m_S(0, 0) = c_*(b_*cos(theta) - d_*sin(theta));
                m_S(0, 1) = c_*(b_*cos(theta) + d_*sin(theta));
                m_S(1, 0) = c_*(b_*sin(theta) + d_*cos(theta));
                m_S(1, 1) = c_*(b_*sin(theta) - d_*cos(theta));

                m_S(2, 0) = -c_;
                m_S(2, 1) = c_;
                m_S(3, 0) = 1.0;
                m_S(4, 1) = 1.0;

                m_S_dot(0, 0) = c_*(-b_*sin(theta) - d_*cos(theta));
                m_S_dot(0, 1) = c_*(-b_*sin(theta) + d_*cos(theta));
                m_S_dot(1, 0) = c_*(b_*cos(theta) - d_*sin(theta));
                m_S_dot(1, 1) = c_*(b_*cos(theta) + d_*sin(theta));

                m_S_dot *= v(2);
            }
        }

        const Eigen::Vector3d & RobotWrapper::com(const Data & data) const
        {
            return data.com[0];
        }

        Eigen::VectorXd  RobotWrapper::nonLinearEffects(const Data & data) 
        {   
            if (mobile_flag_){
            //    cout << "ori" << data.nle.transpose().tail(7) << endl;
                return m_S.transpose() * data.nle;                 
            }
            return data.nle;
        }

        const SE3 & RobotWrapper::position(const Data & data, const Model::JointIndex index) const
        {
            assert(index<data.oMi.size());
            return data.oMi[index];
        }

        const Motion & RobotWrapper::velocity(const Data & data, const Model::JointIndex index) const
        {
            assert(index<data.v.size());
            return data.v[index];
        }

        const Motion & RobotWrapper::acceleration(const Data & data, const Model::JointIndex index) const
        {
            assert(index<data.a.size());
            return data.a[index];
        }

        const Eigen::MatrixXd & RobotWrapper::mass(const Data & data)
        {            
            if (mobile_flag_){
                m_M = m_S.transpose() * data.M * m_S;
            }
            else
                m_M = data.M;
            return m_M;
        }

        const Eigen::MatrixXd & RobotWrapper::mass_inverse(const Data & data){
            m_Minv = this->mass(data).inverse();
            return m_Minv;
        }

        const Eigen::MatrixXd & RobotWrapper::coriolis(const Data & data){
            m_C = data.C;
            return m_C;
        }

        void RobotWrapper::jacobianWorld(const Data & data, const Model::JointIndex index, Data::Matrix6x & J) 
        {   
            Data::Matrix6x J_tmp(6, this->nv());
            if (mobile_flag_){
                assert(index<m_model.frames.size());
                m_Rot.topLeftCorner(3, 3) = this->position(data, index).rotation();
                m_Rot.bottomRightCorner(3, 3) = this->position(data, index).rotation();
                pinocchio::getJointJacobian(m_model, data, index, pinocchio::LOCAL, J_tmp);
                J = m_Rot * J_tmp;// * m_S;
            }
            else{
                return pinocchio::getJointJacobian(m_model, data, index, pinocchio::WORLD, J) ;
            }
        }

        SE3 RobotWrapper::framePosition(const Data & data, const Model::FrameIndex index) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            return data.oMi[f.parent].act(f.placement);
        }

        void RobotWrapper::framePosition(const Data & data, const Model::FrameIndex index, SE3 & framePosition) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            framePosition = data.oMi[f.parent].act(f.placement);
        }

        Motion RobotWrapper::frameVelocity(const Data & data, const Model::FrameIndex index) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            return f.placement.actInv(data.v[f.parent]);
        }
    
        void RobotWrapper::frameVelocity(const Data & data, const Model::FrameIndex index, Motion & frameVelocity) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            frameVelocity = f.placement.actInv(data.v[f.parent]);
        }
    
        Motion RobotWrapper::frameAcceleration(const Data & data, const Model::FrameIndex index) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            return f.placement.actInv(data.a[f.parent]);
        }

        void RobotWrapper::frameAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            frameAcceleration = f.placement.actInv(data.a[f.parent]);
        }

        Motion RobotWrapper::frameClassicAcceleration(const Data & data, const Model::FrameIndex index) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            Motion a = f.placement.actInv(data.a[f.parent]);
            Motion v = f.placement.actInv(data.v[f.parent]);
            a.linear() += v.angular().cross(v.linear());
            return a;
        }

        void RobotWrapper::frameClassicAcceleration(const Data & data, const Model::FrameIndex index, Motion & frameAcceleration) const
        {
            assert(index<m_model.frames.size());
            const Frame & f = m_model.frames[index];
            frameAcceleration = f.placement.actInv(data.a[f.parent]);
            Motion v = f.placement.actInv(data.v[f.parent]);
            frameAcceleration.linear() += v.angular().cross(v.linear());
        }

        void RobotWrapper::frameJacobianLocal(Data & data, const Model::FrameIndex index, Data::Matrix6x & J) 
        {
            Data::Matrix6x J_tmp(6, this->nv());
            assert(index<m_model.frames.size());

            if (mobile_flag_){
                pinocchio::getFrameJacobian(m_model, data, index, pinocchio::LOCAL, J_tmp);
                J = J_tmp * m_S;
            }
            else{
                return pinocchio::getFrameJacobian(m_model, data, index, pinocchio::LOCAL, J) ;
            }
        }



    }
}
