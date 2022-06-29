#include "kimm_hqp_controller/formulation/inverse_dynamics_formulation_base.hpp"

namespace kimmhqp{
    TaskLevel::TaskLevel(tasks::TaskBase & t, unsigned int priority):
    task(t),
    priority(priority)
    {}

  TaskLevelForce::TaskLevelForce(tasks::TaskContactForce & task,
                                unsigned int priority):
    task(task),
    priority(priority)
  {}
  
  InverseDynamicsFormulationBase::InverseDynamicsFormulationBase(const std::string & name, RobotWrapper & robot, bool verbose)
  : m_name(name)
  , m_robot(robot)
  , m_verbose(verbose)
  {}
  // bool InverseDynamicsFormulationBase::addRigidContact(ContactBase & contact)
  // {
  //   return addRigidContact(contact, 1e-5);
  // }
}
