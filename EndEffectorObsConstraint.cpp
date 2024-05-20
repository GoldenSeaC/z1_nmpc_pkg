#include <ocs2_mobile_manipulator/MobileManipulatorPreComputation.h>
#include <ocs2_mobile_manipulator/constraint/EndEffectorObtConstraint.h>

namespace ocs2{
namespace mobile_manipulator{

EndEffectorObtConstraint::EndEffectorObtConstraint(std::vector<std::tuple<scalar_t,vector3_t>> obstacles,const EndEffectorKinematics<scalar_t>& endEffectorKinematics, const ReferenceManager& referenceManager,const scalar_t d_safe):
        StateConstraint(ConstraintOrder::Linear),   
        obstacle_positions_(obstacles),
        endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
        referenceManagerPtr_(&referenceManager),
        d_safe_(d_safe) {
    v_d_safe_<<d_safe,d_safe,d_safe;
    if (endEffectorKinematics.getIds().size() != 1) {
        throw std::runtime_error("[EndEffectorConstraint] endEffectorKinematics has wrong number of end effector IDs.");
    }
    pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());
}

size_t EndEffectorObtConstraint::getNumConstraints(scalar_t time)const{ // 改为所有关节的避障
    return obstacle_positions_.size();//只有position 约束
}

vector_t EndEffectorObtConstraint::getValue(scalar_t time,const vector_t& state,const PreComputation& preComputation)const{
    if(pinocchioEEKinPtr_!=nullptr){
        const auto& preCompMM=cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());      
    }
    //设置constraints //假设是立方体约束 first step 指针对第一个障 所以下面长度为 1
    vector_t constraint(obstacle_positions_.size());
    //考虑障碍
    int i=0;
    for(auto obs:obstacle_positions_){
        scalar_t radios_ob=std::get<0>(obs);
        Eigen::Matrix<scalar_t, 3, 1> obstacle_i=std::get<1>(obs);
        constraint(i)=(endEffectorKinematicsPtr_->getPosition(state).front()-obstacle_i).norm()-d_safe_-radios_ob;
        i++;
    }
    return constraint;
}

VectorFunctionLinearApproximation EndEffectorObtConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComputation) const{
    //hx 函数：||eepos-obspos||^2-d_safe^2 =(eepos_x-obspos_x)^2+(eepos_y-obspos_y)^2+(eepos_z-obspos_z)^2-d_safe^2
    //first oder derivative of state: 二范数的求导 结果应该是长度为state 所有关节角度的列向量 
    if(pinocchioEEKinPtr_!=nullptr){
        const auto& preCompMM=cast<MobileManipulatorPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());      
    }
    auto approximation= VectorFunctionLinearApproximation(obstacle_positions_.size(),state.rows(),0);
    const auto eePosition = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    // vector_t input(6);
    // vector3_t state_dot=endEffectorKinematicsPtr_->getVelocity(state,input.setZero()).front();
    
    //获取雅可比的上三行
    matrix_t J_pos=eePosition.dfdx;//这里时不是要换成endEffectorKinematicsPtr_
    vector_t h_dot=J_pos.transpose()*(state.head<3>()-std::get<1>(obstacle_positions_.front()));
    Eigen::Matrix<scalar_t, 3, 1> obstacle_1=std::get<1>(obstacle_positions_.front());
    scalar_t norm_down=(endEffectorKinematicsPtr_->getPosition(state).front()-obstacle_1).norm();
    approximation.f=this->getValue(time,state,preComputation);
    //fill the dfdx matrix with obs_row x state_clo
    matrix_t obs_dfx(obstacle_positions_.size(),state.rows());
    for(int i=0;i<obstacle_positions_.size();i++){
        obs_dfx.row(i)=(h_dot*(1/norm_down)).transpose();
    }
    approximation.dfdx=obs_dfx;
    approximation.dfdu=approximation.dfdx;
    approximation.dfdu.setZero();
    // std::cout<<"obs!! linear approximaion \n"<<approximation.dfdx<<std::endl;
    return approximation;
}

}
}