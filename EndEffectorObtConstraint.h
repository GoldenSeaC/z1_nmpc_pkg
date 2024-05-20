#pragma once

#include <memory>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <tuple>
namespace ocs2 {
namespace mobile_manipulator
{   
    class EndEffectorObtConstraint final:public StateConstraint{//这里要用的是inequalityconstraint 所以最终添加的时候用relaxbarrierfunction 形式 非cbf 形式直接用距离直接添加到 StateSoftConstraint 的constraint项
        public:
            using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
            using quaternion_t = Eigen::Quaternion<scalar_t>;
            EndEffectorObtConstraint(std::vector<std::tuple<scalar_t,vector3_t>> obstacles,const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const ReferenceManager& referenceManager,const scalar_t d_safe);
            ~EndEffectorObtConstraint() override=default;
            EndEffectorObtConstraint* clone() const override{return new EndEffectorObtConstraint(obstacle_positions_,*endEffectorKinematicsPtr_, *referenceManagerPtr_,d_safe_);}

            size_t getNumConstraints(scalar_t time) const override;
            vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComputation) const override;
            //阶次为 StateConstraint(ConstraintOrder::Linear), 所以在StateSoftConstraint 的二次近似其是用的是如下的一次近似
            VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const override;

        private:
            //tuple : radios / position
            std::vector<std::tuple<scalar_t,vector3_t>> obstacle_positions_;//
              /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
            PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;
            std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
            const ReferenceManager* referenceManagerPtr_;
            scalar_t d_safe_;
            vector3_t v_d_safe_;
    };
} // namespace mobile_manipulator
}