#ifndef __task_motion_hpp__
#define __task_motion_hpp__

#include "kimm_hqp_controller/tasks/task_base.hpp"
#include "kimm_hqp_controller/trajectory/trajectory_base.hpp"

namespace kimmhqp
{
  namespace tasks
  {
    class TaskMotion : public TaskBase
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        typedef math::Vector Vector;
        typedef trajectory::TrajectorySample TrajectorySample;

        TaskMotion(const std::string & name, RobotWrapper & robot);

        virtual const TrajectorySample & getReference() const;

        virtual const Vector & getDesiredAcceleration() const;

        virtual Vector getAcceleration(ConstRefVector dv) const;

        virtual const Vector & position_error() const;
        virtual const Vector & velocity_error() const;
        virtual const Vector & position() const;
        virtual const Vector & velocity() const;
        virtual const Vector & position_ref() const;
        virtual const Vector & velocity_ref() const;

        virtual void setMask(math::ConstRefVector mask);
        virtual const Vector & getMask() const;
        virtual bool hasMask();

      protected:
        Vector m_mask;
        Vector m_dummy;
        bool m_mobile;
        trajectory::TrajectorySample TrajectorySample_dummy;
    };
  }
}

#endif 