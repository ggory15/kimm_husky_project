#include "kimm_hqp_controller/tasks/task_joint_bound.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"

namespace kimmhqp
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectory;
    using namespace pinocchio;

    TaskTorqueBounds::TaskTorqueBounds(const std::string & name,
                                     RobotWrapper & robot):
      TaskMotion(name, robot),
      m_constraint(name, robot.nv()),
      m_nv(robot.nv()),
      m_na(robot.na()),
      m_mobile(robot.ismobile())
    {
      if (!m_mobile){
        int offset = m_nv-m_na;
        for(int i=0; i<offset; i++)
        {
          m_constraint.upperBound()(i) = 1e10;
          m_constraint.lowerBound()(i) = -1e10;
        }
      }
      else{
        m_constraint.resize(m_nv-3, m_nv-3);
      }
    }

    int TaskTorqueBounds::dim() const
    { return m_nv; }

    void TaskTorqueBounds::setTorqueBounds(ConstRefVector lower, ConstRefVector upper)
    {
      assert(lower.size()==m_na);
      assert(upper.size()==m_na);
      m_torque_lb = lower;
      m_torque_ub = upper;
    }

    const Vector & TaskTorqueBounds::getTorqueLowerBounds() const{
      return m_torque_lb;
    }
    const Vector & TaskTorqueBounds::getTorqueUpperBounds() const{
      return m_torque_ub;
    }

    const ConstraintBase & TaskTorqueBounds::getConstraint() const
    {
      return m_constraint;
    }

    void TaskTorqueBounds::setMask(ConstRefVector mask)
    {
      m_mask = mask;
    }

    const ConstraintBase & TaskTorqueBounds::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector v,
                                                    Data &)
    {
      int offset = m_nv-m_na;
      if (m_mobile)
        offset = 0;

      for(int i=0; i<m_na; i++)
      {
      	
        // TODO: use mask here
        	m_constraint.upperBound()(offset+i) = m_torque_ub(i); //std::min(m_ddq_max_due_to_vel(i), m_a_ub(i));
        	m_constraint.lowerBound()(offset+i) = m_torque_lb(i); // std::max(m_ddq_min_due_to_vel(i), m_a_lb(i));
      	
      }
            
      return m_constraint;
    }


    TaskJointBounds::TaskJointBounds(const std::string & name,
                                     RobotWrapper & robot):
      TaskMotion(name, robot),
      m_constraint(name, robot.nv()),
      m_nv(robot.nv()),
      m_na(robot.na()),
      m_mobile(robot.ismobile())
    {
      if (!m_mobile){
        int offset = m_nv-m_na;
        for(int i=0; i<offset; i++)
        {
          m_constraint.upperBound()(i) = 1e10;
          m_constraint.lowerBound()(i) = -1e10;
        }
      }
      else{
        m_constraint.resize(m_nv-3, m_nv-3);
      }

      m_buffer = 5.0 * M_PI/180.0;
    }

    int TaskJointBounds::dim() const
    { return m_nv; }

    void TaskJointBounds::setJointBounds(ConstRefVector lower, ConstRefVector upper)
    {
      assert(lower.size()==m_na);
      assert(upper.size()==m_na);
      m_joint_lb = lower;
      m_joint_ub = upper;
    }

    const Vector & TaskJointBounds::getJointLowerBounds() const{
      return m_joint_lb;
    }
    const Vector & TaskJointBounds::getJointUpperBounds() const{
      return m_joint_ub;
    }

    const ConstraintBase & TaskJointBounds::getConstraint() const
    {
      return m_constraint;
    }

    void TaskJointBounds::setMask(ConstRefVector mask)
    {
      m_mask = mask;
    }

    const ConstraintBase & TaskJointBounds::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data &)
    {
      using namespace std;

      int offset = m_nv-m_na;
      double m_Kp = 4000.0; 
      double m_Kd = sqrt(4000.0);

      if (m_mobile){
        for (int i=0; i<2; i++){
          m_constraint.upperBound()(i) = m_joint_ub(i);
          m_constraint.lowerBound()(i) = m_joint_lb(i);
        }
        for (int i=2; i<m_nv-3; i++){   
         // cout << m_joint_lb.transpose() << endl;
          if (q(i+3) < m_joint_lb(i) + m_buffer) {
					  m_constraint.lowerBound()(i) = m_Kp * ((m_joint_lb(i) + m_buffer) - q(i+3) ) - m_Kd * v(i+3) ;
					  if (m_constraint.lowerBound()(i) > 200.0)
						  m_constraint.lowerBound()(i) = 200.0;

					  m_constraint.upperBound()(i) = 200.0;
				  }
				  else if (q(i+3) > m_joint_ub(i) - m_buffer) {
					  m_constraint.upperBound()(i) = m_Kp * ((m_joint_ub(i) - m_buffer) - q(i+3) ) - m_Kd * v(i+3);
					  if (m_constraint.upperBound()(i) < -200.0)
						  m_constraint.upperBound()(i) = -200.0;

					  m_constraint.lowerBound()(i) = -200.0;
				  }
				  else {
					  m_constraint.upperBound()(i) = 200.0;
					  m_constraint.lowerBound()(i) = -200.0;
				  }
        }
      }
      else{
        for (int i = 0; i < m_robot.nv(); i++) {
				  if (q(i) < m_joint_lb(i) + m_buffer) {
					  m_constraint.lowerBound()(i) = m_Kp * ((m_joint_lb(i) + m_buffer) - q(i)) - m_Kd * v(i);
					  m_constraint.upperBound()(i) = 200.0;
				  }
				  else if (q(i) > m_joint_ub(i) - m_buffer) {
					  m_constraint.upperBound()(i) = m_Kp * ((m_joint_ub(i) - m_buffer) - q(i)) - m_Kd * v(i);
					  m_constraint.lowerBound()(i) = -200.0;
				  }
				  else {
					  m_constraint.upperBound()(i) = 200.0;
					  m_constraint.lowerBound()(i) = -200.0;
				  }
			  }
      }
      
      return m_constraint;
    }
    
  }
}
