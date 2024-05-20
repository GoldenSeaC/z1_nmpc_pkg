//文件说明:机械臂控制的ros接口 
//订阅话题:z1_joint_angle msg:mpc_state （来自real robot）node
//       :mpc policy(来自mpc节点)
//发布话题:要用到的primalsolution（转化为msg：mpc_state ）发给real robot node
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_msgs/mpc_state.h>
#include <ocs2_msgs/reset.h>
#include <ocs2_msgs/mpc_flattened_controller.h>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

#include <iostream>
using namespace ocs2;
using namespace mobile_manipulator;

bool policyReceivedEver_=false;
bool stateReceivedEver_=false;
double mpc_first_request_time=0.0;//recording the first time recieveing the policy
int mpc_state_size,mpc_input_size;

ocs2_msgs::mpc_state solution_next_dt_;//end state + next velocity
SystemObservation cur_observation_;//sending to the visualizer
  //initialize data
SystemObservation initObservation;//init state to reset mpc node
ocs2_msgs::mpc_observation mpcObservationMsg_;//send to the mpc node


std::string real_state_topic="z1_joint_angle";
std::string topicPrefix_="mobile_manipulator";
std::string real_cmd_topic="z1_cmd";
auto commandPtr = std::make_unique<ocs2::CommandData>();
auto primalSolutionPtr = std::make_unique<ocs2::PrimalSolution>();

void real_state_callback(const ocs2_msgs::mpc_state::ConstPtr &real_state);
void readPolicyMsg(const ocs2_msgs::mpc_flattened_controller& msg, ocs2::CommandData& commandData,
                                      ocs2::PrimalSolution& primalSolution, ocs2::PerformanceIndex& performanceIndices);
void policy_callback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg);                

int main(int argc, char** argv) {
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string taskFile, libFolder, urdfFile;
  nodeHandle.getParam("/taskFile", taskFile);
  nodeHandle.getParam("/libFolder", libFolder);
  nodeHandle.getParam("/urdfFile", urdfFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::cerr << "Loading library folder: " << libFolder << std::endl;
  std::cerr << "Loading urdf file: " << urdfFile << std::endl;
  // Robot Interface
  mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder, urdfFile);
  // MRT
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&interface.getRollout());
  // mrt.launchNodes(nodeHandle);//接收控制策略

  //topic and service needed
  ros::Subscriber z1_state_subscriber=nodeHandle.subscribe(real_state_topic,10,&real_state_callback);//接收实体状态
  ros::Subscriber mpcPolicySubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_flattened_controller>(topicPrefix_+"_mpc_policy",1,&policy_callback);
  ros::ServiceClient mpcResetServiceClient_=nodeHandle.serviceClient<ocs2_msgs::reset>(topicPrefix_+"_mpc_reset");//重启mpc
  ros::Publisher z1_cmd_publisher=nodeHandle.advertise<ocs2_msgs::mpc_state>(real_cmd_topic,10);
  ros::Publisher mpcObservationPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_observation>(topicPrefix_ + "_mpc_observation", 1);

  //initial observation and target
  mpc_input_size=interface.getManipulatorModelInfo().inputDim;
  mpc_state_size=interface.getManipulatorModelInfo().stateDim;
  solution_next_dt_.value.resize(mpc_input_size+mpc_state_size);
  std::cout<<"solution data size:"<<mpc_state_size<<"+"<<mpc_input_size<<"="<<solution_next_dt_.value.size()<<std::endl;
  
  initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);//

  vector_t initTarget(7);
  initTarget.head(3) << 0.3, 0.3, 0.3;
  initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
  const vector_t zeroInput = vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
  const TargetTrajectories initTargetTrajectories({initObservation.time}, {initTarget}, {zeroInput});
  

  //RESET MPC NODE WAIT AND WAIT FOR THE FIRST POLICY COMMAND 重启mpc节点并等待第一个控制策略
  ocs2_msgs::reset resetService;
  resetService.request.reset = static_cast<uint8_t>(true);
  resetService.request.targetTrajectories = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(initTargetTrajectories);
  while (!mpcResetServiceClient_.waitForExistence(ros::Duration(5.0)) && ::ros::ok() && ::ros::master::check()) {
      ROS_ERROR_STREAM("Failed to call service to reset MPC, retrying...");
  }
  mpcResetServiceClient_.call(resetService);
  ROS_INFO_STREAM("MPC node has been reset.");

  // Visualization
  auto dummyVisualization = std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(nodeHandle, interface);
  ros::Rate loop_rate(100);
  while(ros::ok()){
        //update data to rviz
        //update cmd
        if(!policyReceivedEver_||!stateReceivedEver_){//等待第一次收到mpc控制策略和 机械臂状态

            mpcObservationMsg_=ros_msg_conversions::createObservationMsg(initObservation);
            // mrt.setCurrentObservation(initObservation);//由于没有mrt.launchNode 所以没有作用
            mpc_first_request_time=ros::Time::now().toSec();
            // std::cout<<"Wait for the initial policy time :"<<mpc_first_request_time<<std::endl;
            //pub cmd
            mpcObservationPublisher_.publish(mpcObservationMsg_);//state observation
            z1_cmd_publisher.publish(solution_next_dt_);
        }
        else{
          mpcObservationMsg_=ros_msg_conversions::createObservationMsg(cur_observation_);//
          mpcObservationPublisher_.publish(mpcObservationMsg_);//state observation
          z1_cmd_publisher.publish(solution_next_dt_);
        }
        if(stateReceivedEver_&&policyReceivedEver_)
          dummyVisualization->update(cur_observation_,*primalSolutionPtr,*commandPtr);
        loop_rate.sleep();
        ros::spinOnce();
  }
}


void real_state_callback(const ocs2_msgs::mpc_state::ConstPtr &real_state){//tranform real state to mpc_observation msg and SystemObservation
  //update current observation and init obseravtion
  Eigen::Matrix<double,6,1> state_cur;
  for(int i=0;i<real_state->value.size();i++){
    state_cur(i)=real_state->value[i];
    // cur_observation_.state(i)=real_state->value[i];
  }
  cur_observation_.state=state_cur;
  if(!policyReceivedEver_)//update inital state until recieving the policy
      initObservation.state=state_cur;
  cur_observation_.time=ros::Time::now().toSec()-mpc_first_request_time;
  stateReceivedEver_=true;
}

void readPolicyMsg(const ocs2_msgs::mpc_flattened_controller& msg, ocs2::CommandData& commandData,
                                      ocs2::PrimalSolution& primalSolution, ocs2::PerformanceIndex& performanceIndices) {
  commandData.mpcInitObservation_ = ocs2::ros_msg_conversions::readObservationMsg(msg.initObservation);
  commandData.mpcTargetTrajectories_ = ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(msg.planTargetTrajectories);
  performanceIndices = ocs2::ros_msg_conversions::readPerformanceIndicesMsg(msg.performanceIndices);

  const size_t N = msg.timeTrajectory.size();
  if (N == 0) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] controller message is empty!");
  }
  if (msg.stateTrajectory.size() != N && msg.inputTrajectory.size() != N) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] state and input trajectories must have same length!");
  }
  if (msg.data.size() != N) {
    throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Data has the wrong length!");
  }

  primalSolution.clear();

  primalSolution.modeSchedule_ = ocs2::ros_msg_conversions::readModeScheduleMsg(msg.modeSchedule);

  ocs2::size_array_t stateDim(N);
  ocs2::size_array_t inputDim(N);
  primalSolution.timeTrajectory_.reserve(N);
  primalSolution.stateTrajectory_.reserve(N);
  primalSolution.inputTrajectory_.reserve(N);
  for (size_t i = 0; i < N; i++) {
    stateDim[i] = msg.stateTrajectory[i].value.size();
    inputDim[i] = msg.inputTrajectory[i].value.size();
    primalSolution.timeTrajectory_.emplace_back(msg.timeTrajectory[i]);
    primalSolution.stateTrajectory_.emplace_back(
        Eigen::Map<const Eigen::VectorXf>(msg.stateTrajectory[i].value.data(), stateDim[i]).cast<ocs2::scalar_t>());
    primalSolution.inputTrajectory_.emplace_back(//所以这里的inputTrafectory 应该是 vector<Eigen::VectorXf> 内部的Eigen::VectorXf 包含了系统的输入
        Eigen::Map<const Eigen::VectorXf>(msg.inputTrajectory[i].value.data(), inputDim[i]).cast<ocs2::scalar_t>());
  }//二维向量的形式PUT MSG-> TIME AND EACH STATE INFO -> PRIMALSOLUTION

  primalSolution.postEventIndices_.reserve(msg.postEventIndices.size());
  for (auto ind : msg.postEventIndices) {
    primalSolution.postEventIndices_.emplace_back(static_cast<size_t>(ind));
  }

  std::vector<std::vector<float> const*> controllerDataPtrArray(N, nullptr);
  for (int i = 0; i < N; i++) {
    controllerDataPtrArray[i] = &(msg.data[i].data);
  }

  // instantiate the correct controller
  switch (msg.controllerType) {
    case ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD: {
      auto controller = ocs2::FeedforwardController::unFlatten(primalSolution.timeTrajectory_, controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(new ocs2::FeedforwardController(std::move(controller)));
      break;
    }
    case ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR: {
      auto controller = ocs2::LinearController::unFlatten(stateDim, inputDim, primalSolution.timeTrajectory_, controllerDataPtrArray);
      primalSolution.controllerPtr_.reset(new ocs2::LinearController(std::move(controller)));
      break;
    }
    default:
      throw std::runtime_error("[MRT_ROS_Interface::readPolicyMsg] Unknown controllerType!");
  }
}
void policy_callback(const ocs2_msgs::mpc_flattened_controller::ConstPtr& msg){
  std::cout<<"policy recieved"<<std::endl;  
      // read new policy and command from msg
  auto performanceIndicesPtr = std::make_unique<ocs2::PerformanceIndex>();
  readPolicyMsg(*msg, *commandPtr, *primalSolutionPtr, *performanceIndicesPtr);
  //得到了PRIMALSOLUTION
  // std::cout<<"recieved command at time "<<mpc_request_time<<std::endl;
  cur_observation_.input=primalSolutionPtr->inputTrajectory_[1];//update observation

  //设置数据顺序为position + velocity  
  for(int i=0; i<mpc_state_size;i++){
      auto fianl_state=primalSolutionPtr->stateTrajectory_.front()*0.7+primalSolutionPtr->stateTrajectory_.back()*0.3;//建议mpc 预测时间短一点
      solution_next_dt_.value[i]=static_cast<float>(fianl_state[i]);//position
  }
  for(int i=0; i<mpc_input_size;i++)
      solution_next_dt_.value[mpc_state_size+i]=static_cast<float>(primalSolutionPtr->inputTrajectory_[0][i]);//velocity 即input的当前值
  //trans primalsolution to joint torque command
  policyReceivedEver_=true;
}