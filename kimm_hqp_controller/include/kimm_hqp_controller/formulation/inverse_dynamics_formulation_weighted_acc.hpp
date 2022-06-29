#ifndef __inv_dyn_acc_w_hpp__
#define __inv_dyn_acc_w_hpp__

#include "kimm_hqp_controller/formulation/inverse_dynamics_formulation_base.hpp"
#include "kimm_hqp_controller/math/constraint_equality.hpp"
#include "kimm_hqp_controller/math/constraint_inequality.hpp"
#include "kimm_hqp_controller/math/constraint_bound.hpp"
#include "kimm_hqp_controller/formulation/contact_level.hpp"

#include <vector>

namespace kimmhqp{

    class InverseDynamicsFormulationWeightedAccForce: public InverseDynamicsFormulationBase
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef pinocchio::Data Data;
            typedef math::Vector Vector;
            typedef math::Matrix Matrix;
            typedef math::ConstRefVector ConstRefVector;
            typedef tasks::TaskBase TaskBase;
            typedef tasks::TaskMotion TaskMotion;
            typedef solver::HQPOutput HQPOutput;
            typedef tasks::TaskContactForce TaskContactForce;
            typedef solver::WHQPData WHQPData;

            InverseDynamicsFormulationWeightedAccForce(const std::string & name, RobotWrapper & robot, bool verbose=false);

            Data & data() ;

            unsigned int nVar() const;
            unsigned int nEq() const;
            unsigned int nIn() const;

            bool addMotionTask(TaskMotion & task, Eigen::VectorXd weight, unsigned int priorityLevel, double transition_duration=0.0);
            bool addMotionTask(TaskMotion & task, double weight, unsigned int priorityLevel, double transition_duration=0.0);

            bool updateTaskWeight(const std::string & task_name, Eigen::VectorXd weight);
            bool updateTaskWeight(const std::string & task_name, double weight);

            bool removeTask(const std::string & taskName, double transition_duration=0.0);

            const WHQPData & computeProblemData(double time, ConstRefVector q, ConstRefVector v);

            const Vector & getAccelerations(const HQPOutput & sol);

            const Vector & getActuatorForces(const HQPOutput & sol);


        public:

            template<class TaskLevelPointer>
            void addTask(TaskLevelPointer task, Eigen::VectorXd weight, unsigned int priorityLevel);
            template<class TaskLevelPointer>
            void addTask(TaskLevelPointer task, double weight, unsigned int priorityLevel);

            void resizeHqpData();

            bool removeFromHqpData(const std::string & name);

            bool decodeSolution(const HQPOutput & sol);

            void resetHqpData();

            Data m_data;
            WHQPData m_hqpData;
            std::vector<std::shared_ptr<TaskLevel> >        m_taskMotions;

            double m_t;         /// time
            unsigned int m_v;   /// number of acceleration variables
            unsigned int m_k;
            unsigned int m_eq;  /// number of equality constraints
            unsigned int m_in;  /// number of inequality constraints
            Matrix m_Jc;        /// contact force Jacobian
            std::shared_ptr<math::ConstraintEquality> m_baseDynamics;

            bool m_solutionDecoded;
            Vector m_dv;
            Vector m_f;
            Vector m_tau;

            bool m_mobile;
        };

    
}

#endif