#include"ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <thread>
#include "unitree_arm_sdk/control/unitreeArm.h"

//ocs2 msgs 
#include <ocs2_msgs/mpc_flattened_controller.h>
#include <ocs2_msgs/reset.h>
#include <ocs2_msgs/mpc_state.h>

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"
#include "sensor_msgs/JointState.h"

#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include "ocs2_mpc/CommandData.h"
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

#include <ocs2_msgs/reset.h>
#include <ocs2_mpc/SystemObservation.h>

#include "communication_thread.h"
using namespace UNITREE_ARM;

ocs2::SystemObservation sys_state;
ocs2_msgs::mpc_observation mpcObservationMsg_;
ocs2_msgs::mpc_state z1_state_;//real robot state
std::vector<double> mpc_solution_;

std::string Node_name="mpc_to_real";
std::string real_state_topic="z1_joint_angle";
std::string policy_topic="z1_cmd";
bool ever_recieved_solution=false;
void policy_rcv_callback(const ocs2_msgs::mpc_state::ConstPtr& rcv_policy);

int main(int argc, char *argv[]){
    ros::init(argc, argv,Node_name);
    ros::NodeHandle nh;

    std::cout << std::fixed << std::setprecision(3);
    bool hasGripper = false;
    unitreeArm arm(hasGripper);//不控制夹抓
    z1_interface z1_arm(arm);
    z1_arm.back_to_start();//测试并回到原位

    //开启通信 使能机械臂
    std::thread unitreeArm_cmuct(&z1_interface::data_swap_Run,&z1_arm);
    std::this_thread::sleep_for(std::chrono::microseconds(300));//default 500Hz9
    
    ros::Publisher z1_state_publisher=nh.advertise<ocs2_msgs::mpc_state>(real_state_topic,10);
    //订阅policy
    ros::Subscriber z1_mpc_solution_subscriber=nh.subscribe<ocs2_msgs::mpc_state>(policy_topic,10,&policy_rcv_callback);

    //接收policy 并用sdk发送
    ros::Rate loop_rate(500);
    while(ros::ok()){
        //transform the state vector of unitree to ocs2_msgs
        vector_t joint_state=z1_arm.rcv_cur_state;
        z1_state_.value.resize(joint_state.rows());
        for(int i=0;i<joint_state.rows();i++){
            std::cout<<" "<<i<<":"<<joint_state(i)<<std::endl;
            z1_state_.value[i] =static_cast<float>(joint_state(i));
        }
        if(ever_recieved_solution)
            z1_arm.update_joint_target(mpc_solution_);
        z1_state_publisher.publish(z1_state_);
        std::cout<<"real state published"<<std::endl;
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    unitreeArm_cmuct.join();
}

void policy_rcv_callback(const ocs2_msgs::mpc_state::ConstPtr& rcv_policy){
    std::cout<<"solution data size: "<<rcv_policy->value.size()<<std::endl;
    std::vector<double> mpc_solution;
    for(auto value:rcv_policy->value){
        mpc_solution.push_back(value);
        std::cout<<" "<<value;
    }
    mpc_solution_.swap(mpc_solution);
    ever_recieved_solution=true;
    std::cout<<" "<<std::endl;
}