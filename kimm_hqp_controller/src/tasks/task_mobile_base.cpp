#include "kimm_hqp_controller/tasks/task_mobile_base.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"
#include "kimm_hqp_controller/math/util.hpp"
#include <Eigen/SVD>

using namespace std;
namespace kimmhqp
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectory;
    using namespace pinocchio;

    TaskMobileEquality::TaskMobileEquality(const std::string & name,
                                     RobotWrapper & robot,
                                     const bool & isorictrl,
                                     unsigned int index):
      TaskMotion(name, robot),
      m_constraint(name, 3, robot.nv()-3),
      m_ref(12, 6),
      m_ori_ctrl(isorictrl),
      m_mobile(robot.ismobile()),
      m_index(index)
    {
      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_p_error_vec.setZero(6);
      m_v_error_vec.setZero(6);
      m_p.resize(12);
      m_v.resize(6);
      m_p_ref.resize(12);
      m_v_ref_vec.resize(6);
      m_Kp.setZero(6);
      m_Kd.setZero(6);
      m_a_des.setZero(6);
     
      m_J.setZero(6, robot.nv()-3);
      m_J_rotated.setZero(6, robot.nv()-3);

      assert(m_mobile == true);

      m_mask.resize(3);
      m_mask.fill(1.);
      setMask(m_mask);

      if (m_mobile)
        m_local_frame = false;
      else
        m_local_frame = true;
    }

    void TaskMobileEquality::setMask(math::ConstRefVector mask)
    {
      TaskMotion::setMask(mask);
      int n = dim();
      m_constraint.resize(n, (unsigned int)m_J.cols());

      m_p_error_masked_vec.resize(n);
      m_v_error_masked_vec.resize(n);
      m_drift_masked.resize(n);
      m_a_des_masked.resize(n);
    }

    int TaskMobileEquality::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskMobileEquality::Kp() const { return m_Kp; }

    const Vector & TaskMobileEquality::Kd() const { return m_Kd; }

    void TaskMobileEquality::Kp(ConstRefVector Kp)
    {
      //assert(Kp.size()==3);
      m_Kp.head(3) = Kp.head(3);
      m_Kp.tail(3) = Kp.head(3);
    }

    void TaskMobileEquality::Kd(ConstRefVector Kd)
    {
      //assert(Kd.size()==3);
      m_Kd.head(3) = Kd.head(3);
      m_Kd.tail(3) = Kd.head(3);
    }

    void TaskMobileEquality::setReference(TrajectorySample & ref)
    {
      m_ref = ref;
      vectorToSE3(ref.pos, m_M_ref);
      m_v_ref = Motion(ref.vel);
      m_a_ref = Motion(ref.acc);
    }

    const TrajectorySample & TaskMobileEquality::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskMobileEquality::position_error() const
    {
      return m_p_error_masked_vec;
    }

    const Vector & TaskMobileEquality::velocity_error() const
    {
      return m_v_error_masked_vec;
    }

    const Vector & TaskMobileEquality::position() const
    {
      return m_p;
    }

    const Vector & TaskMobileEquality::velocity() const
    {
      return m_v;
    }

    const Vector & TaskMobileEquality::position_ref() const
    {
      return m_p_ref;
    }

    const Vector & TaskMobileEquality::velocity_ref() const
    {
      return m_v_ref_vec;
    }

    const Vector & TaskMobileEquality::getDesiredAcceleration() const
    {
      return m_a_des_masked;
    }

    Vector TaskMobileEquality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv + m_drift_masked;
    }

    const ConstraintBase & TaskMobileEquality::getConstraint() const
    {
      return m_constraint;
    }

    void TaskMobileEquality::useLocalFrame(bool local_frame)
    {
      m_local_frame = local_frame;
    }

    const ConstraintBase & TaskMobileEquality::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    Data & data)
    {
     
      SE3 oMi;
      Motion v_frame;
      Index m_frame_id = m_robot.model().getFrameId("panda_joint1");
      
      m_robot.getMobilePosition(data, m_index, m_frame_id, oMi);
      m_robot.getMobileVelocity(data, m_index, m_frame_id, v_frame);
      m_robot.getMobileClassicAcceleration(data, m_index, m_frame_id, m_drift);
      
      m_robot.getMobileJacobianLocal(data, m_index, m_frame_id, m_J); // 6 by 9 (for husky with single franka arm)
      m_J.topRightCorner(6, 7).setZero();

      errorInSE3(oMi, m_M_ref, m_p_error);          // pos err in local frame
      SE3ToVector(m_M_ref, m_p_ref);
      SE3ToVector(oMi, m_p);

     
      // Transformation from local to world
      m_wMl.rotation(oMi.rotation());

      if (m_local_frame) {
        m_p_error_vec = m_p_error.toVector();
        m_v_error =  m_wMl.actInv(m_v_ref) - v_frame;  // vel err in local frame

        // desired acc in local frame
        m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
                  + m_Kd.cwiseProduct(m_v_error.toVector())
                  + m_wMl.actInv(m_a_ref).toVector();
      } 
      else {
        m_p_error_vec = m_wMl.toActionMatrix() *   // pos err in local world-oriented frame
            m_p_error.toVector();

        m_v_error = m_v_ref - m_wMl.act(v_frame);  // vel err in local world-oriented frame

        m_drift = m_wMl.act(m_drift);

        // desired acc in local world-oriented frame
        m_a_des = m_Kp.cwiseProduct(m_p_error_vec)
                  + m_Kd.cwiseProduct(m_v_error.toVector())
                  + m_a_ref.toVector();

        // Use an explicit temporary `m_J_rotated` here to avoid allocations.
        m_J_rotated.noalias() = m_wMl.toActionMatrix() * m_J;
        m_J = m_J_rotated;
      }

      //cout << "m_a_des" << m_a_des.transpose(I) << endl;

      m_v_error_vec = m_v_error.toVector();
      m_v_ref_vec = m_v_ref.toVector();
      m_v = v_frame.toVector();

      
      
      int idx = 0;
      
      if (m_ori_ctrl){
        for (int i = 3; i < 6; i++) {
          //  if (m_mask(i-3) != 1.) continue;
            
            m_constraint.matrix().row(idx) = m_J.row(i);
            m_constraint.vector().row(idx) = (m_a_des - m_drift.toVector()).row(i);
            m_a_des_masked(idx)            = m_a_des(i);
            m_drift_masked(idx)            = m_drift.toVector()(i);
            m_p_error_masked_vec(idx)      = m_p_error_vec(i);
            m_v_error_masked_vec(idx)      = m_v_error_vec(i);
      
            idx += 1;
        }
      }
      else{
        for (int i = 0; i < 3; i++) {
            if (m_mask(i) != 1.) continue;
            
            m_constraint.matrix().row(idx) = m_J.row(i);
            m_constraint.vector().row(idx) = (m_a_des - m_drift.toVector()).row(i);
            m_a_des_masked(idx)            = m_a_des(i);
            m_drift_masked(idx)            = m_drift.toVector()(i);
            m_p_error_masked_vec(idx)      = m_p_error_vec(i);
            m_v_error_masked_vec(idx)      = m_v_error_vec(i);
      
            idx += 1;
        }
      }
     
      return m_constraint;
    }
    
  }
}
