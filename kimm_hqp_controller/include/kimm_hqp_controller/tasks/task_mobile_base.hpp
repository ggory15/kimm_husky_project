#ifndef __task_mobile_equality_hpp__
#define __task_mobile_equality_hpp__

#include "kimm_hqp_controller/tasks/task_motion.hpp"
#include "kimm_hqp_controller/math/constraint_equality.hpp"
#include "kimm_hqp_controller/trajectory/trajectory_base.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace kimmhqp
{
  namespace tasks
  {
    class TaskMobileEquality : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef trajectory::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;
      typedef pinocchio::Data::Matrix6x Matrix6x;
      typedef pinocchio::Motion Motion;
      typedef pinocchio::SE3 SE3;

      TaskMobileEquality(const std::string & name,
                      RobotWrapper & robot,
                      const bool & isorictrl,
                      unsigned int index = 5);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      /** Return the desired task acceleration (after applying the specified mask).
       *  The value is expressed in local frame is the local_frame flag is true,
       *  otherwise it is expressed in a local world-oriented frame.
      */
      const Vector & getDesiredAcceleration() const;

      /** Return the task acceleration (after applying the specified mask).
       *  The value is expressed in local frame is the local_frame flag is true,
       *  otherwise it is expressed in a local world-oriented frame.
      */
      Vector getAcceleration(ConstRefVector dv) const;

      virtual void setMask(math::ConstRefVector mask);

      /** Return the position tracking error (after applying the specified mask).
       *  The error is expressed in local frame is the local_frame flag is true,
       *  otherwise it is expressed in a local world-oriented frame.
      */
      const Vector & position_error() const;

      /** Return the velocity tracking error (after applying the specified mask).
       *  The error is expressed in local frame is the local_frame flag is true,
       *  otherwise it is expressed in a local world-oriented frame.
      */
      const Vector & velocity_error() const;

      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector & Kp() const;
      const Vector & Kd() const;
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

       /**
       * @brief Specifies if the jacobian and desired acceloration should be
       * expressed in the local frame or the local world-oriented frame.
       *
       * @param local_frame If true, represent jacobian and acceloration in the
       *   local frame. If false, represent them in the local world-oriented frame.
       */
      void useLocalFrame(bool local_frame);

    protected:

      bool m_ori_ctrl;
      unsigned int m_index;
      Motion m_p_error, m_v_error;
      Vector m_p_error_vec, m_v_error_vec;
      Vector m_p_error_masked_vec, m_v_error_masked_vec;
      Vector m_p, m_v;
      Vector m_p_ref, m_v_ref_vec;
      Motion m_v_ref, m_a_ref;
      SE3 m_M_ref, m_wMl;
      Vector m_Kp;
      Vector m_Kd;
      Vector m_a_des, m_a_des_masked;
      Motion m_drift;
      Vector m_drift_masked;
      Matrix6x m_mat;
      Vector m_vec;
      Matrix6x m_J;
      Matrix6x m_J_rotated;
      ConstraintEquality m_constraint;
      TrajectorySample m_ref;
      bool m_local_frame;
      bool m_mobile;
    };

  }
}

#endif 