#ifndef __contact_point_hpp__
#define __contact_point_hpp__

#include "kimm_hqp_controller/contact/contact_base.hpp"
#include "kimm_hqp_controller/tasks/task_se3_equality.hpp"
#include "kimm_hqp_controller/math/constraint_inequality.hpp"
#include "kimm_hqp_controller/math/constraint_equality.hpp"

namespace kimmhqp
{
  namespace contacts
  {
    class ContactPoint : public ContactBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::ConstRefMatrix ConstRefMatrix;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::Matrix3x Matrix3x;
      typedef math::Vector6 Vector6;
      typedef math::Vector3 Vector3;
      typedef math::Vector Vector;
      typedef tasks::TaskSE3Equality TaskSE3Equality;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::SE3 SE3;

      ContactPoint(const std::string & name,
                RobotWrapper & robot,
                const std::string & frameName,
                ConstRefVector contactNormal,
                const double frictionCoefficient,
                const double minNormalForce,
                const double maxNormalForce);

      /// Return the number of motion constraints
      virtual unsigned int n_motion() const;

      /// Return the number of force variables
      virtual unsigned int n_force() const;

      virtual const ConstraintBase & computeMotionTask(const double t,
                                                       ConstRefVector q,
                                                       ConstRefVector v,
                                                       Data & data);

      virtual const ConstraintInequality & computeForceTask(const double t,
                                                            ConstRefVector q,
                                                            ConstRefVector v,
                                                            const Data & data);

      virtual const Matrix & getForceGeneratorMatrix();

      virtual const ConstraintEquality & computeForceRegularizationTask(const double t,
                                                                        ConstRefVector q,
                                                                        ConstRefVector v,
                                                                        const Data & data);

      const TaskSE3Equality & getMotionTask() const;
      const ConstraintBase & getMotionConstraint() const;
      const ConstraintInequality & getForceConstraint() const;
      const ConstraintEquality & getForceRegularizationTask() const;
      double getMotionTaskWeight() const;
      const Matrix3x & getContactPoints() const;

      double getNormalForce(ConstRefVector f) const;
      double getMinNormalForce() const;
      double getMaxNormalForce() const;

      const Vector & Kp();  // cannot be const because it set a member variable inside
      const Vector & Kd();  // cannot be const because it set a member variable inside
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

      bool setContactNormal(ConstRefVector contactNormal);

      bool setFrictionCoefficient(const double frictionCoefficient);
      bool setMinNormalForce(const double minNormalForce);
      bool setMaxNormalForce(const double maxNormalForce);
      bool setMotionTaskWeight(const double w);
      void setReference(const SE3 & ref);
      void setForceReference(ConstRefVector & f_ref);
      void setRegularizationTaskWeightVector(ConstRefVector & w);

      /**
       * @brief Specifies if properties of the contact point and motion task
       * are expressed in the local or local world oriented frame. The contact
       * forces, contact normal and contact coefficients are interpreted in
       * the specified frame.
       *
       * @param local_frame If true, use the local frame, otherwise use the
       * local world oriented
       */
      void useLocalFrame(bool local_frame);

    protected:

      void updateForceInequalityConstraints();
      void updateForceRegularizationTask();
      void updateForceGeneratorMatrix();

      TaskSE3Equality m_motionTask;
      ConstraintInequality m_forceInequality;
      ConstraintEquality m_forceRegTask;
      Vector3 m_contactNormal;
      Vector3 m_fRef;
      Vector3 m_weightForceRegTask;
      Matrix3x m_contactPoints;
      Vector m_Kp3, m_Kd3;  // gain vectors to be returned by reference
      double m_mu;
      double m_fMin;
      double m_fMax;
      double m_regularizationTaskWeight;
      double m_motionTaskWeight;
      Matrix m_forceGenMat;
    };
  }
}

#endif // ifndef __invdyn_contact_6d_hpp__