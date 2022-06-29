#ifndef __task_joint_posture_hpp__
#define __task_joint_posture_hpp__

#include "kimm_hqp_controller/tasks/task_motion.hpp"
#include "kimm_hqp_controller/math/constraint_equality.hpp"
#include "kimm_hqp_controller/trajectory/trajectory_base.hpp"

namespace kimmhqp
{
  namespace tasks
  {
    class TaskJointPosture : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectory::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::VectorXi VectorXi;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;

      TaskJointPosture(const std::string & name,
                      RobotWrapper & robot);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;

      const Vector & mask() const; // deprecated
      void mask(const Vector & mask); // deprecated
      virtual void setMask(math::ConstRefVector mask);

      const Vector & position_error() const;
      const Vector & velocity_error() const;
      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector m_Kp;
      Vector m_Kd;
      Vector m_p_error, m_v_error;
      Vector m_p, m_v;
      Vector m_a_des;
      VectorXi m_activeAxes;
      TrajectorySample m_ref;
      bool m_mobile;
      ConstraintEquality m_constraint;
    };

  }
}

#endif 