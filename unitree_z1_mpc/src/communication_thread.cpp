#include "communication_thread.h"
#include <thread>
z1_interface::z1_interface(unitreeArm &arm):z1_arm_(arm){
    //back to start
}

void z1_interface::back_to_start(){
    z1_enable=true;
    z1_arm_.sendRecvThread->start();

    z1_arm_.backToStart();
    z1_arm_.setFsm(ArmFSMState::PASSIVE);
    z1_arm_.setFsm(ArmFSMState::LOWCMD);

    std::vector<double> KP, KW;
    KP = z1_arm_._ctrlComp->lowcmd->kp;
    KW = z1_arm_._ctrlComp->lowcmd->kd;
    z1_arm_._ctrlComp->lowcmd->setControlGain(KP, KW);
    z1_arm_.sendRecvThread->shutdown();

    Vec6 initQ = z1_arm_.lowstate->getQ();
    double duration = 600;
    Vec6 targetQ;
    targetQ << 0, 1.0, -0.7, -0.4, 0, 0;
    Timer timer(z1_arm_._ctrlComp->dt);
    for(int i(0); i<duration; i++){
        z1_arm_.q = initQ * (1-i/duration) + targetQ * (i/duration);
        z1_arm_.qd = (targetQ - initQ) / (duration * z1_arm_._ctrlComp->dt);
        z1_arm_.tau = z1_arm_._ctrlComp->armModel->inverseDynamics(z1_arm_.q, z1_arm_.qd, Vec6::Zero(), Vec6::Zero());
        
        z1_arm_.setArmCmd(z1_arm_.q, z1_arm_.qd, z1_arm_.tau);
        z1_arm_.sendRecv();
        timer.sleep();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1000));//default 500Hz
    initQ << 0, 0.5, -0.3, -0.18, 0, 0;
        for(int i(0); i<duration; i++){
        z1_arm_.q = targetQ * (1-i/duration) + initQ * (i/duration);
        z1_arm_.qd = (initQ - targetQ) / (duration * z1_arm_._ctrlComp->dt);
        z1_arm_.tau = z1_arm_._ctrlComp->armModel->inverseDynamics(z1_arm_.q, z1_arm_.qd, Vec6::Zero(), Vec6::Zero());
        
        z1_arm_.setArmCmd(z1_arm_.q, z1_arm_.qd, z1_arm_.tau);
        z1_arm_.sendRecv();
        timer.sleep();
    }
    std::cout <<"|| test run over ||"<<std::endl;
    //back

    z1_arm_.sendRecvThread->start();
    z1_arm_.setFsm(ArmFSMState::PASSIVE);
    z1_arm_.sendRecvThread->shutdown();
    std::this_thread::sleep_for(std::chrono::microseconds(200));//default 500Hz
}

void z1_interface::update_joint_target(std::vector<double> solution){
    //size check
    std::cout<<"transmit solution to cmd: "<<solution.size()<<std::endl;
    if(solution.size()==12){
        vector_t cmd_velocity(6);
        vector_t cmd_state(6);
        for(int i=0;i<6;i++){
            cmd_state(i)=solution[i];
            cmd_velocity(i)=solution[i+6]*0.4;
        }
        z1_arm_.q = cmd_state;
        z1_arm_.qd =cmd_velocity;
        z1_arm_.tau = z1_arm_._ctrlComp->armModel->inverseDynamics(z1_arm_.q, z1_arm_.qd, Vec6::Zero(), Vec6::Zero());//控制量为速度 那么给角度应该给什么呢(最优控制轨迹的末尾)
    }
}
void z1_interface::data_swap_Run(){

    z1_arm_.sendRecvThread->start();
    z1_arm_.setFsm(ArmFSMState::PASSIVE);
    z1_arm_.setFsm(ArmFSMState::LOWCMD);
    Vec6 initQ;
    initQ << 0, 0.5, -0.3, -0.18, 0, 0;
    z1_arm_.q=initQ;
    z1_arm_.qd=Vec6::Zero();
    z1_arm_.tau=Vec6::Zero();
    z1_arm_.setArmCmd(z1_arm_.q, z1_arm_.qd, z1_arm_.tau);

    while (z1_enable)
    {
        // std::cout<<"in communication thread"<<std::endl;
        z1_arm_.setArmCmd(z1_arm_.q, z1_arm_.qd, z1_arm_.tau); //暂时不发送 进查看状态
        z1_arm_.sendRecv();
        rcv_cur_state=z1_arm_.lowstate->getQ();//获取当前关节角度 需要发送出去
        std::this_thread::sleep_for(std::chrono::microseconds(2));//default 500Hz
        // std::cout<<"in communication thread"<<std::endl;

    }
    
    z1_arm_.setFsm(ArmFSMState::JOINTCTRL);
    z1_arm_.backToStart();
    z1_arm_.setFsm(ArmFSMState::PASSIVE);
    z1_arm_.sendRecvThread->shutdown();
}
void z1_interface::run_over(){
    std::cout<<"restart sendRecv"<<std::endl;
    z1_arm_.sendRecvThread->start();
    z1_arm_.setFsm(ArmFSMState::JOINTCTRL);
    z1_arm_.backToStart();
    z1_arm_.setFsm(ArmFSMState::PASSIVE);
    z1_arm_.sendRecvThread->shutdown();
}