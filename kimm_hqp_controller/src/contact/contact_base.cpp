#include "kimm_hqp_controller/contact/contact_base.hpp"

namespace kimmhqp
{
  namespace contacts
  {
    ContactBase::ContactBase(const std::string & name,
                             RobotWrapper & robot):
      m_name(name),
      m_robot(robot)
    {}

    const std::string & ContactBase::name() const
    {
      return m_name;
    }

    void ContactBase::name(const std::string & name)
    {
      m_name = name;
    }

  }
}