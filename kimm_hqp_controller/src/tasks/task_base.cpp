#include "kimm_hqp_controller/tasks/task_base.hpp"

namespace kimmhqp
{
  namespace tasks
  {
    TaskBase::TaskBase(const std::string & name,
                       RobotWrapper & robot):
      m_name(name),
      m_robot(robot)
    {}

    const std::string & TaskBase::name() const
    {
      return m_name;
    }

    void TaskBase::name(const std::string & name)
    {
      m_name = name;
    }
    
  }
}
