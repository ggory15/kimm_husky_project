#include "kimm_hqp_controller/tasks/task_motion.hpp"

namespace kimmhqp
{
  namespace tasks
  {
    typedef math::Vector Vector;
    typedef trajectory::TrajectorySample TrajectorySample;

    TaskMotion::TaskMotion(const std::string & name,
                           RobotWrapper & robot):
      TaskBase(name, robot)
    {}
    
    void TaskMotion::setMask(math::ConstRefVector mask)
    {
      m_mask = mask;
    }

    bool TaskMotion::hasMask()
    {
      return m_mask.size() > 0;
    }

    const Vector & TaskMotion::getMask() const { return m_mask; }

    const TrajectorySample & TaskMotion::getReference() const { return TrajectorySample_dummy; }

    const Vector & TaskMotion::getDesiredAcceleration() const  { return m_dummy; }

    Vector TaskMotion::getAcceleration(ConstRefVector ) const  { return m_dummy; }

    const Vector & TaskMotion::position_error() const { return m_dummy; }
    const Vector & TaskMotion::velocity_error() const  { return m_dummy; }
    const Vector & TaskMotion::position() const  { return m_dummy; }
    const Vector & TaskMotion::velocity() const  { return m_dummy; }
    const Vector & TaskMotion::position_ref() const  { return m_dummy; }
    const Vector & TaskMotion::velocity_ref() const  { return m_dummy; }    
  }
}
