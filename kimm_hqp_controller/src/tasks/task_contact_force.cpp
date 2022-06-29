#include "kimm_hqp_controller/tasks/task_contact_force.hpp"

namespace kimmhqp
{
  namespace tasks
  {
    TaskContactForce::TaskContactForce(const std::string & name,
                                       kimmhqp::robot::RobotWrapper & robot):
      TaskBase(name, robot)
    {}
  }
}
