//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/WeightedWbc.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/Float64MultiArray.h"
#include "legged_controllers/utilities.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/rnea.hpp>
#include "ocs2_centroidal_model/FactoryFunctions.h"

// #include "rbdl/rbdl.h"
// #include "rbdl/Model.h"
// #include "rbdl/Kinematics.h"
// #include "rbdl/Dynamics.h"
// #include "rbdl/addons/urdfreader/urdfreader.h"
namespace legged
{
//创建了controller_loader节点后，使用controller_manger包的spawner选择控制器
//spawner选择控制器LeggedController后会进行依次进行init、starting，然后在循环中进行update
bool LeggedController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) // 重写了原先ros控制器的init，controller_nh句柄通过ros调用
{
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);    //controller_nh句柄从one_start.launch中读取路径，然后将路径赋值给字符串
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);

  bool verbose = false;//打印参数，false不打印
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useRollout", useRollout_);
  
  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo()); // 质心模型和Pinocchio模型的映射关系
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      leggedInterface_->getPinocchioInterface(), pinocchioMapping, leggedInterface_->modelSettings().contactNames3DoF);   //足端数据接口
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);    //足端轨迹、质心轨迹等可视化
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));    //自碰撞体积可视化

  //*********************感控*********************//
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
      *dynamic_cast<SwitchedModelReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getSwingTrajectoryPlanner(),
      leggedInterface_->getCentroidalModelInfo().numThreeDofContacts, nh);
  //********************************************//

  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());    //刚体模型接口，mpc转wbc会用
  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();   //电机接口
  std::vector<std::string> joint_names{
      "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint", "leg_l6_joint",
      "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint", "leg_r6_joint",
      "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint", "zarm_l5_joint", "zarm_l6_joint","zarm_l7_joint",
      "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint", "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint"};
  for (const auto& joint_name : joint_names)
  {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));   //电机控制器实现
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("imu_link");  //imu接口

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  bool useHierarchicalWbc = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useHierarchicalWbc", useHierarchicalWbc);

  // 四元数关节模型
  pinocchioInterfaceQuatPtr_ = std::make_unique<PinocchioInterface>(
      centroidal_model::createPinocchioInterfaceFree(urdfFile, leggedInterface_->modelSettings().jointNames));
  
  if (!useHierarchicalWbc)
    wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(),
                                         leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_,
                                         *pinocchioInterfaceQuatPtr_);
  else
    wbc_ = std::make_shared<HierarchicalWbc>(leggedInterface_->getPinocchioInterface(),
                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_,
                                             *pinocchioInterfaceQuatPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);
  wbc_->setStanceMode(true);
  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  // Configuring the hardware interface
  eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

  // Reading relevant parameters
  RetrievingParameters();   //方便用的一些量

  // loadEigenMatrix
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);    //从配置文件加载默认关节位置

  // Configuring an inverse kinematics processing object
  inverseKinematics_.setParam(std::make_shared<PinocchioInterface>(leggedInterface_->getPinocchioInterface()),
                              std::make_shared<CentroidalModelInfo>(leggedInterface_->getCentroidalModelInfo()));     //逆运动学接口

  // RigidBodyDynamics::Model model_rbdl;
  // RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path, &model_rbdl, true);

  // std::cout << "**********q_size**********" << std::endl;
  // std::cout << model_rbdl.q_size << std::endl;
  // std::cout << "**********qdot_size**********" << std::endl;
  // std::cout << model_rbdl.qdot_size << std::endl;

  return true;
}

void LeggedController::starting(const ros::Time& time)
{
  startingTime_.fromSec(time.toSec() - 0.0001);   //启动时刻
  const ros::Time shifted_time = time - startingTime_;
  // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;       //初始状态
  currentObservation_.mode = ModeNumber::STANCE;
  
  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });    //初始参考轨迹

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);

  mpcRunning_ = false;

  // Mode Subscribe 订阅模式话题
  ModeSubscribe();

  // Dynamic server  PD控制器相关
  serverPtr_ =
      std::make_unique<dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>>(ros::NodeHandle("controller"));    //rqt的controller话题
  dynamic_reconfigure::Server<legged_controllers::TutorialsConfig>::CallbackType f;
  f = boost::bind(&LeggedController::dynamicParamCallback, this, _1, _2);
  serverPtr_->setCallback(f);
}

void LeggedController::update(const ros::Time& time, const ros::Duration& period)
{
  const ros::Time shifted_time = time - startingTime_;    //当前时刻-启动时刻 = 系统运行的时刻
  // State Estimate
  updateStateEstimation(shifted_time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);   //把估计后的新状态打包成ros消息，通过线程发布

  // Evaluate the current policy
  vector_t optimizedState(stateDim_), optimizedInput(inputDim_);    //MPC的优化状态和输入

  size_t plannedMode = 0;
  bool mpc_updated_ = false;
  if (firstStartMpc_)
  {
    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();
    if(useRollout_)   //验证轨迹用
      mpcMrtInterface_->rolloutPolicy(currentObservation_.time, currentObservation_.state, leggedInterface_->rolloutSettings().timeStep,
                                    optimizedState, optimizedInput, plannedMode);  
    else
      mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState,
                                         optimizedInput, plannedMode);  //直接使用当前时刻下的解(线性插值后)
    currentObservation_.input = optimizedInput;   //当前输入

    const auto &modeSequence = leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getModeScheduleSelf().modeSequence;
    const auto &eventTimes = leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getModeScheduleSelf().eventTimes;
    mpc_updated_ = true;
  }


  if (setWalkFlag_)
  {
    wbc_->setStanceMode(false);
  }
  else      //没有发布setwalk话题的时候不会介入MPC，仅WBC维持站立姿态
  {
    if(!load_initFlag_){
      optimizedState.setZero();
      optimizedInput.setZero();
      optimizedState.segment(6, 6) = currentObservation_.state.segment<6>(6); //机体位置
      optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;  //关节位置
      current_base_pose = currentObservation_.state.segment<6>(6);
      
    }
    else{
      //记录当前位置
      optimizedState.setZero();
      optimizedInput.setZero();   //反作用力、关节速度
      optimizedState.segment(6, 6) = currentObservation_.state.segment<6>(6); // 机体位置
      // optimizedState.segment(8, 1) = current_base_pose.segment<1>(2);                                               // 机体位置
      optimizedState.segment(6 + 6, jointDim_) = stateEstimate_->getRbdState().segment(6, jointDim_); // 关节位置
      // optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;                                 // 用的话关节就会被锁死了
      if(hand_reset_flg){
        optimizedState.segment(6 + 6 + 12, 6) = defalutJointPos_.segment<6>(12) * 2 * sinf64(timeh);
        optimizedState.segment(6 + 6 + 12, 14) = optimizedState.segment(6 + 6 + 12, 14).setOnes() * 1.1 * sinf64(timeh);
        optimizedState.segment(6 + 6 + 12 + 7, 7) = -optimizedState.segment(6 + 6 + 12 + 7, 7);
        timeh += 0.008;
        // optimizedState.segment(6 + 6 + 12, 14) = defalutJointPos_.segment<14>(12);
      }
#if !USE_6_AXIS_HAND
      else
        optimizedState.segment(6 + 6 + 12, 14) = defalutJointPos_.segment<14>(12);
#endif
      if (body_reset_flg)
      {
          optimizedState.segment(9, 3).setZero(); // 机体姿态
      }
      // optimizedState.segment(9, 3).setZero(); // 机体姿态

    }
                
    plannedMode = 3;  //双脚站立
    wbc_->setStanceMode(true);
  }
  // optimizedState = [hG,qb,qj]  optimizedInput = [f,vj]
  
  const vector_t& mpc_planned_body_pos = optimizedState.segment(6, 6);
  const vector_t& mpc_planned_joint_pos = optimizedState.segment(6 + 6, jointDim_);
  const vector_t& mpc_planned_joint_vel = optimizedInput.segment(12, jointDim_);
  // std::cout << "*****************current base pose*****************" << std::endl;
  // std::cout << currentObservation_.state.segment<6>(6) << std::endl;
  
  // WBC
  wbcTimer_.startTimer();
  wbc_->init_flg = load_initFlag_;
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec(), currentObservation_);
  const vector_t& wbc_planned_torque = x.tail(jointDim_);
  const vector_t& wbc_planned_joint_acc = x.segment(6, jointDim_);
  const vector_t& wbc_planned_body_acc = x.head(6);
  const vector_t& wbc_planned_contact_force = x.segment(6 + jointDim_, wbc_->getContactForceSize());
  wbcTimer_.endTimer();

  // std::cout << "wbc_planned_contact_force = " << std::endl;
  // std::cout << wbc_planned_contact_force << std::endl;
  auto cpose = optimizedState;
  auto cvel = optimizedInput;

  if(!load_initFlag_){
    posDes_ = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    velDes_ = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
  }
  else{
    cpose.segment(6, 6) = currentObservation_.state.segment<6>(6);
    // cpose.segment(9, 3).setZero();
    cpose.segment(6 + 6, jointDim_) = stateEstimate_->getRbdState().segment(6, jointDim_);
    posDes_ = centroidal_model::getJointAngles(cpose, leggedInterface_->getCentroidalModelInfo());
    velDes_ = centroidal_model::getJointVelocities(cvel, leggedInterface_->getCentroidalModelInfo());
  }

  scalar_t dt = period.toSec();
  posDes_ = posDes_ + 0.5 * wbc_planned_joint_acc * dt * dt;
  velDes_ = velDes_ + wbc_planned_joint_acc * dt;

  vector_t output_torque(jointDim_);

    //*********************** Set Joint Command: Normal Tracking *****************************//
    for (size_t j = 0; j < jointDim_; ++j)
    {
      //"Limit protection
      const auto &model = leggedInterface_->getPinocchioInterface().getModel();
      double lower_bound = model.lowerPositionLimit(6 + j);
      double upper_bound = model.upperPositionLimit(6 + j);
      // if (!emergencyStopFlag_ && loadControllerFlag_ &&
      //     (hybridJointHandles_[j].getPosition() > upper_bound + 0.02 ||
      //      hybridJointHandles_[j].getPosition() < lower_bound - 0.02))
      // {
      //   emergencyStopFlag_ = true;
      //   std::cerr << "Reach Position Limit!!!!!!!!!!!!!!!!!!!!!!!! " << j << ":" << hybridJointHandles_[j].getPosition()
      //             << std::endl;
      // }
      if (!loadControllerFlag_) // 没有发布控制器话题的时候按照位控的PD参数
      {
        if (j == 0 || j == 6)
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_stance1, kd_stance1, 0);
        }
        else if (j == 1 || j == 7)
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_stance2, kd_stance2, 0);
        }
        else if (j == 2 || j == 8)
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_stance3, kd_stance3, 0);
        }
        else if (j == 3 || j == 9)
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_stance1, kd_stance4, 0);
        }
        else if (j == 4 || j == 10)
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_stance4, kd_stance5, 0);
        }
        else if (j == 5 || j == 11)
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_stance5, kd_stance6, 0);
        }
        else if ((j >= 12 && j <= 17) || (j >= 19 && j <= 24))
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], 20.0, 1.01, 0);
        }
        else if (j == 18 || j == 25)
        {
          hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], 5.0, 0.2, 0);
        }
      }
      else
      { // 发布控制话题后按照力控PD参数
        contact_flag_t cmdContactFlag = modeNumber2StanceLeg(
            mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
        if (j == 0 || j == 6)
        {
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                            cmdContactFlag[int(j / 6)] ? 350 : 350, kd_mpc1,
                                            wbc_planned_torque(j));
        }
        else if (j == 1 || j == 7)
        {
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                            cmdContactFlag[int(j / 6)] ? kp_mpc2 : kp_mpc1, kd_mpc1,
                                            wbc_planned_torque(j));
        }
        else if (j == 2 || j == 3 || j == 8 || j == 9)
        {
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                            cmdContactFlag[int(j / 6)] ? 400 : 400, kd_mpc1,
                                            wbc_planned_torque(j));
        }
        else if (j == 4 || j == 10)
        {
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                            cmdContactFlag[int(j / 6)] ? kp_mpc4 : kp_mpc4, kd_mpc2,
                                            wbc_planned_torque(j));
        }
        else if (j == 5 || j == 11)
        {
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                            cmdContactFlag[int(j / 6)] ? 250 : 250, 1.5,
                                            wbc_planned_torque(j));
        }
        else if ((j >= 12 && j <= 17) || (j >= 19 && j <= 24))
        {
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                            20.0, 1.0,
                                            wbc_planned_torque(j));
        }
        else if (j == 18 || j == 25)
        {
          hybridJointHandles_[j].setCommand(posDes_[j], velDes_[j],
                                            5.0, 0.1,
                                            wbc_planned_torque(j));
          
        }
      }
      if (emergencyStopFlag_) // 急停
      {
        hybridJointHandles_[j].setCommand(0, 0, 0, 1, 0);
      }
      posDesOutput_(j) = hybridJointHandles_[j].getPositionDesired();
      velDesOutput_(j) = hybridJointHandles_[j].getVelocityDesired();
      
      output_torque(j) = hybridJointHandles_[j].getFeedforward() +
                         hybridJointHandles_[j].getKp() *
                             (hybridJointHandles_[j].getPositionDesired() - hybridJointHandles_[j].getPosition()) +
                         hybridJointHandles_[j].getKd() *
                             (hybridJointHandles_[j].getVelocityDesired() - hybridJointHandles_[j].getVelocity());
    }
  //*********************** Set Joint Command: Torque Tracking Test *****************************//

  CommandData command_data;
  vector_t planned_state = currentObservation_.state;
  vector_t planned_input = currentObservation_.input;
  planned_state.tail(jointDim_) = posDesOutput_;
  planned_input.tail(jointDim_) = velDesOutput_;
  command_data.mpcTargetTrajectories_.timeTrajectory.push_back(currentObservation_.time);
  command_data.mpcTargetTrajectories_.stateTrajectory.push_back(planned_state);
  command_data.mpcTargetTrajectories_.inputTrajectory.push_back(planned_input);
  command_data = mpc_updated_ ? mpcMrtInterface_->getCommand() : command_data;
  PrimalSolution primal_solution = mpc_updated_ ? mpcMrtInterface_->getPolicy() : PrimalSolution();

  // if(!load_initFlag_){
  //   measuredRbdStateTraj_.reserve(primal_solution.stateTrajectory_.size());
  //   measuredRbdStateTraj_.clear();
  // }else{
  //   measuredRbdStateTraj_.push_back(measuredRbdState_);
  //   measuredRbdStateTraj_.resize(primal_solution.stateTrajectory_.size());
  // }
  // Visualization
  robotVisualizer_->update(currentObservation_, primal_solution, command_data,
                           leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner(),
                           wbc_->getLinearMomentumDesired(),
                           /*measuredRbdStateTraj_*/ measuredRbdState_, wbc_->get_dcm_des(), wbc_->get_dcm_mea(),
                           wbc_->get_foot_des(), wbc_->get_foot_mea());
  selfCollisionVisualization_->update(currentObservation_);

  //感控
  footPlacementVisualizationPtr_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period)
{
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()),
      jointTor(hybridJointHandles_.size());
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t cmdContactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)   //关节更新
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTor(i) = hybridJointHandles_[i].getEffort();
  }

  cmdContactFlag = modeNumber2StanceLeg(
      mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));    //根据当前系统时间，查询当前行走模式下对应的步态，在根据步态得到接触点触地的情况
  if (!firstStartMpc_)    //如果MPC未启动,接触点全部设为true，即STANCE步态(双脚着地)
  {
    for (size_t i = 0; i < 8; ++i)
    {
      cmdContactFlag[i] = true;
    }
  }
  stateEstimate_->updateCmdContact(cmdContactFlag);   //将更新的接触点状态赋给观测器接口
  stateEstimate_->setStartStopTime4Legs(
      leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->threadSaftyGetStartStopTime(
          currentObservation_.time));   //得到足端轨迹的起始和结束时间(参考轨迹有起点时刻和终点时刻，可能有关系)

  for (size_t i = 0; i < 4; ++i)    //四元数更新
  {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }

  for (size_t i = 0; i < 3; ++i)    //角速度和加速度更新
  {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }

  for (size_t i = 0; i < 9; ++i)    //协方差更新
  {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel); // 更新的关节赋给观测器接口
  stateEstimate_->updateContact(cmdContactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance,
                            linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period); // 更新位置和线速度（刚体动力学），足端位置

  currentObservation_.time = time.toSec();    //MPC时间更新
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state.head(stateDim_) = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);   //刚体动力学状态转质心动力学状态
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();   //根据接触点情况获得当前步态，并更新mpc用的步态模式

  const auto& reference_manager = leggedInterface_->getSwitchedModelReferenceManagerPtr();
  reference_manager->getSwingTrajectoryPlanner()->setBodyVelWorld(stateEstimate_->getBodyVelWorld());
  reference_manager->setEstContactFlag(cmdContactFlag);

  stateEstimate_->setCmdTorque(jointTor);
  stateEstimate_->estContactForce(period);  //估计接触力******

  auto remove_gravity = linearAccel;
  remove_gravity(2) -= 9.81;
}

LeggedController::~LeggedController()
{
  controllerRunning_ = false;
  mpcRunning_ = false;
  if (mpcThread_.joinable())
  {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                            const std::string& referenceFile, bool verbose)
{
  // 创建LeggedInterface的接口，LeggedInterface为机器人的配置接口，需要urdf、task.info、reference.info三个配置文件
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupGridMap();
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose); //配置该机器人的优化相关参数
}

void LeggedController::setupMpc()   //MPC设置
{
  // mpc_ = std::make_shared<GaussNewtonDDP_MPC>(leggedInterface_->mpcSettings(), leggedInterface_->ddpSettings(),leggedInterface_->getRollout(),
  //                                             leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer()); // 使用DDP求解
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer()); // 使用SQP求解

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  auto rosReferenceManagerPtr =
      std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);    //订阅2个话题，步态和参考轨迹
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr); // mpc订阅步态和参考轨迹话题
  // 创建一个发布者，发布legged_robot_mpc_observation话题，该话题会发布MPC的求解信息
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);

  //*********************感控*********************//
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nh, dynamic_cast<LeggedInterface &>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<LeggedInterface &>(*leggedInterface_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation"); // 接收地形信息
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void LeggedController::setupMrt()   //参考轨迹跟踪器
{
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout()); //初始化跟踪器数据(rollout为滚动优化计算的系统输入)
  mpcTimer_.reset();
  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_)      //启动MPC线程
    {
      try
      {
        executeAndSleep(
            [&]() {
              if (mpcRunning_)
              {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();   //更新MPC的策略
                mpcTimer_.endTimer();
                firstStartMpc_ = true;
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  stateEstimate_->loadSettings(taskFile, verbose);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void LeggedController::dynamicParamCallback(legged_controllers::TutorialsConfig& config, uint32_t level)
{
  kp_position = config.kp_position;
  kd_position = config.kd_position;

  kp_big_stance = config.kp_big_stance;
  kp_big_swing = config.kp_big_swing;

  kp_small_stance = config.kp_small_stance;
  kp_small_swing = config.kp_small_swing;
  kd_small = config.kd_small;
  kd_big = config.kd_big;

  kd_feet = config.kd_feet;

  kp_stance1 = config.kp_stance1;
  kp_stance2 = config.kp_stance2;
  kp_stance3 = config.kp_stance3;
  kp_stance4 = config.kp_stance4;
  kp_stance5 = config.kp_stance5;

  kd_stance1 = config.kd_stance1;
  kd_stance2 = config.kd_stance2;
  kd_stance3 = config.kd_stance3;
  kd_stance4 = config.kd_stance4;
  kd_stance5 = config.kd_stance5;
  kd_stance6 = config.kd_stance6;

  kp_mpc1 = config.kp_mpc1;
  kp_mpc2 = config.kp_mpc2;
  kp_mpc3 = config.kp_mpc3;
  kp_mpc4 = config.kp_mpc4;

  kd_mpc1 = config.kd_mpc1;
  kd_mpc2 = config.kd_mpc2;
  kd_mpc3 = config.kd_mpc3;
}

void LeggedController::RetrievingParameters()   
{
  stateDim_ = leggedInterface_->getCentroidalModelInfo().stateDim;                        //状态维度22
  inputDim_ = leggedInterface_->getCentroidalModelInfo().inputDim;                        //输入维度22
  jointDim_ = leggedInterface_->getCentroidalModelInfo().actuatedDofNum;                  //关节维度10
  footDim_ = leggedInterface_->getCentroidalModelInfo().numThreeDofContacts;              //足端维度(xyz)12
  gencoordDim_ = leggedInterface_->getCentroidalModelInfo().generalizedCoordinatesNum;    //广义坐标维度16
  dofPerLeg_ = jointDim_ / 2;   //单条腿的关节维度6 
  defalutJointPos_.resize(jointDim_);   //默认关节位置向量初始化，大小为关节维度12
}

void LeggedController::resetMPC()
{
  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });
  mpcMrtInterface_->resetMpcNode(target_trajectories);
}
void LeggedController::ModeSubscribe()      //订阅话题，根据这些话题执行相应回调函数
{
  subSetWalk_ =
      ros::NodeHandle().subscribe<std_msgs::Float32>("/set_walk", 1, &LeggedController::setWalkCallback, this);

  subLoadcontroller_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/load_controller", 1,
                                                                      &LeggedController::loadControllerCallback, this);
  subEmgstop_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/emergency_stop", 1,
                                                               &LeggedController::EmergencyStopCallback, this);
  subResetTarget_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/reset_estimation", 1,
                                                                  &LeggedController::ResetTargetCallback, this);
  //感控
  subElevationMap_ = ros::NodeHandle().subscribe<std_msgs::Bool>("/start_map", 1,
                                                                    &LeggedController::startElevationMapCallback, this);

  subController_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/start_control", 1,
                                                                  &LeggedController::startControlCallback, this);

  hand_reset_ = ros::NodeHandle().subscribe<std_msgs::Bool>("/hand_reset", 1,
                                                               &LeggedController::handResetCallback, this);
  dcm_seth_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/dcm_seth", 1,
                                                            &LeggedController::dcmHightSetCallback, this);
  dcm_setx_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/dcm_setx", 1,
                                                             &LeggedController::dcmLinearSetCallback, this);
  body_reset_ = ros::NodeHandle().subscribe<std_msgs::Bool>("/body_reset", 1,
                                                            &LeggedController::bodyAngluarSetCallback, this);
}

void LeggedController::EmergencyStopCallback(const std_msgs::Float32::ConstPtr& msg)    //急停回调函数
{
  emergencyStopFlag_ = true;
  ROS_INFO("Successfully load the controller");
}

void LeggedController::setWalkCallback(const std_msgs::Float32::ConstPtr& msg)    //走路回调函数
{
  setWalkFlag_ = true;
  ROS_INFO("Set WALK Mode");
}

void LeggedController::handResetCallback(const std_msgs::Bool::ConstPtr &msg) // 急停回调函数
{
  hand_reset_flg = true;
  ROS_INFO("Successfully reset hand");
}

void LeggedController::dcmHightSetCallback(const std_msgs::Float32::ConstPtr &msg) // 急停回调函数
{
  wbc_->set_dcm_Height(msg->data);
  ROS_INFO("Successfully set DCM h");
}

void LeggedController::bodyAngluarSetCallback(const std_msgs::Bool::ConstPtr &msg)
{
  body_reset_flg = true;
  ROS_INFO("Successfully reset body");
}

void LeggedController::dcmLinearSetCallback(const std_msgs::Float32::ConstPtr &msg)
{
  wbc_->set_dcm_x(msg->data);
  ROS_INFO("Successfully set DCM x");
}

void LeggedController::startElevationMapCallback(const std_msgs::Bool::ConstPtr &msg){
  if(msg->data){
    leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->start_elevation_map_ = true;
    ROS_INFO("elevation start.");
  }
  else{
    leggedInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->start_elevation_map_ = false;
    ROS_WARN("elevation stop.");
  }
}

void LeggedController::startControlCallback(const std_msgs::Float32::ConstPtr &msg)
{
  ros::NodeHandle nh;
  ros::Publisher resset_pub = nh.advertise<std_msgs::Float32>("/reset_estimation", 1);
  ros::Publisher load_pub = nh.advertise<std_msgs::Float32>("/load_controller", 1);
  ros::Publisher walk_pub = nh.advertise<std_msgs::Float32>("/set_walk", 1);
  std_msgs::Float32 msg_float;
  msg_float.data = 1.32;
  if(msg->data == 1.0){
    resset_pub.publish(msg_float);
  }
  else if (msg->data == 2.0){
    load_pub.publish(msg_float);
  }
  else if(msg->data == 3.0){
    walk_pub.publish(msg_float);
  }
}

void LeggedController::loadControllerCallback(const std_msgs::Float32::ConstPtr& msg)   //启动控制器回调函数
{
  loadControllerFlag_ = true;
  mpcRunning_ = true;   //启动MPC
  load_initFlag_ = true;
  ROS_INFO("Successfully load the controller");
}
void LeggedController::ResetTargetCallback(const std_msgs::Float32::ConstPtr& msg)    //重置状态观测值
{
  // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO("Reset the target");
  // loadControllerFlag_ = true;
  // mpcRunning_ = true; // 启动MPC
  // ROS_INFO("Successfully load the controller");
  // setWalkFlag_ = true;
  // ROS_INFO("Set WALK Mode");
} 
}// namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)