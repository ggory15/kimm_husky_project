#ifndef __inv_dyn_base_hpp__
#define __inv_dyn_base_hpp__

#include "kimm_hqp_controller/math/fwd.hpp"
#include "kimm_hqp_controller/robot/robot_wrapper.hpp"
#include "kimm_hqp_controller/tasks/task_motion.hpp"
#include "kimm_hqp_controller/solver/solver_HQP_base.hpp"
#include "kimm_hqp_controller/contact/contact_base.hpp"
#include "kimm_hqp_controller/tasks/task_contact_force.hpp"
//#include "kimm_hqp_controller/solver/util.hpp"

#include <string>

namespace kimmhqp{
    struct TaskLevel
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        tasks::TaskBase & task;
        std::shared_ptr<math::ConstraintBase> constraint;
        unsigned int priority;

        TaskLevel(tasks::TaskBase & t, unsigned int priority);
    };

    struct TaskLevelForce
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        tasks::TaskContactForce & task;
        std::shared_ptr<math::ConstraintBase> constraint;
        unsigned int priority;

        TaskLevelForce(tasks::TaskContactForce & task, unsigned int priority);
    };
    
    class InverseDynamicsFormulationBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef pinocchio::Data Data;
        typedef math::Vector Vector;
        typedef math::RefVector RefVector;
        typedef math::ConstRefVector ConstRefVector;
        typedef tasks::TaskMotion TaskMotion;
        typedef tasks::TaskBase TaskBase;
        typedef solver::HQPData HQPData;
        typedef solver::HQPOutput HQPOutput;
        typedef robot::RobotWrapper RobotWrapper;
        typedef contacts::ContactBase ContactBase;
        typedef tasks::TaskContactForce TaskContactForce;

        InverseDynamicsFormulationBase(const std::string & name, RobotWrapper & robot, bool verbose=false);

        // virtual Data & data() = 0;

        // virtual unsigned int nVar() const = 0;
        // virtual unsigned int nEq() const = 0;
        // virtual unsigned int nIn() const = 0;

        // virtual bool addMotionTask(TaskMotion & task, double weight, unsigned int priorityLevel, double transition_duration=0.0) = 0;

        // virtual bool updateTaskWeight(const std::string & task_name, double weight) = 0;

        // virtual bool removeTask(const std::string & taskName, double transition_duration=0.0) = 0;


        // virtual const HQPData & computeProblemData(double time, ConstRefVector q, ConstRefVector v) = 0;

        // virtual const Vector & getAccelerations(const HQPOutput & sol) = 0;

        // // 
        // virtual bool addForceTask(TaskContactForce & task,
        //                       double weight,
        //                       unsigned int priorityLevel,
        //                       double transition_duration=0.0) = 0;

        // virtual bool addRigidContact(ContactBase & contact);

        // virtual bool addRigidContact(ContactBase & contact, double force_regularization_weight,
        //                          double motion_weight=1.0, unsigned int motion_priority_level=0) = 0;


        // virtual bool updateRigidContactWeights(const std::string & contact_name,
        //                                    double force_regularization_weight,
        //                                    double motion_weight=-1.0) = 0;

        // virtual bool removeRigidContact(const std::string & contactName,
        //                             double transition_duration=0.0) = 0;

        // virtual const Vector & getContactForces(const HQPOutput & sol) = 0;
        // virtual bool getContactForces(const std::string & name, const HQPOutput & sol, RefVector f) = 0;

    protected:
        std::string m_name;
        RobotWrapper m_robot;
        bool m_verbose;
    };
    
}

#endif