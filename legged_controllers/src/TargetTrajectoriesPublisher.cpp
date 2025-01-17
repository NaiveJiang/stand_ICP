//
// Created by qiayuan on 2022/7/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace
{
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;    //设定质心高度
vector_t DEFAULT_JOINT_STATE(10);
scalar_t TIME_TO_TARGET;
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement)    //估计当前位置到目标位置的时间
{
  const scalar_t& dx = desiredBaseDisplacement(0);      //计算线位移
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);    //计算角位移
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;    //角位移/设定角速度 = 旋转时间
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);                 //合位移=x方向位移和y方向位移的合成
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;  //线位移/设定线速度 = 平移时间
  return std::max(rotationTime, displacementTime);    //对比旋转时间和平移时间，选取最大的时间作为当前位置到目标位置的时间
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime)   //目标位置生成参考轨迹
{
  // desired time trajectory
  const scalar_array_t timeTrajectory{ observation.time, targetReachingTime };    //参考时间轨迹：当前时刻到目标时刻之间的时间

  // desired state trajectory   参考状态轨迹
  vector_t currentPose = observation.state.segment<6>(6);     //当前位姿
  scalar_t dz = COM_HEIGHT - currentPose(2);    //高度误差
  dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]); //限幅
  // 这个是参考轨迹的起点，不是当前状态，时刻是对齐的
  currentPose(2) = currentPose(2) + dz;   //将起点设为和终点一样(轨迹直线)，使得当前状态和参考轨迹起点的error变大，能够快速响应
  currentPose(3) = targetPose(3);
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));  //状态轨迹，2行向量，元素是状态空间 状态x=[hG,qb,qj]
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;  //参考轨迹的起点：（线动量,角动量）（起点位置）（关节角度），动量用速度表征，关节角度在摆动腿规划会计算
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;   //参考轨迹的终点

  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));    //输入轨迹，2行向量，元素是输入空间
  return { timeTrajectory, stateTrajectory, inputTrajectory };
}

TargetTrajectories bodyRotationToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                    const scalar_t& targetReachingTime)   //目标姿态生成参考轨迹
{
  // desired time trajectory
  const scalar_array_t timeTrajectory{ observation.time, targetReachingTime };

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  scalar_t dz = COM_HEIGHT - currentPose(2);
  dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]);
  currentPose(2) = currentPose(2) + dz; // 参考轨迹起点，将起点设为和终点一样(轨迹直线)，使得当前状态和参考轨迹起点的error变大，能够快速响应
  currentPose(3) = targetPose(3);
  currentPose(4) = targetPose(4);
  currentPose(5) = targetPose(5);
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
  return { timeTrajectory, stateTrajectory, inputTrajectory };
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation)   //给定位置的参考轨迹生成
{
  const vector_t currentPose = observation.state.segment<6>(6);   //获取当前位置
  const vector_t targetPose = [&]() {   //目标位置
    vector_t target(6);
    target(0) = goal(0);    //目标x、y位置 = 给定x、y位置
    target(1) = goal(1);
    scalar_t dz = COM_HEIGHT - currentPose(2);    //高度变化量 = 质心高度 - 当前高度
    dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]);   //限幅
    target(2) = currentPose(2) + dz;  //目标z位置 = 设定质心高度
    target(3) = goal(3);    //目标yaw = 给定yaw
    target(4) = 0;    //保持pitch和roll为零（机体平衡）
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);    //估计当前位置到目标位置的时间，然后计算到达目标位置的终端时间=当前时间+估计时间
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);   //生成参考轨迹
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation)   //给定速度的参考轨迹生成
{
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);    //欧拉角
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);   //将线速度转换到世界坐标
  const scalar_t timeToTarget = TIME_TO_TARGET;   //到目标速度的时间 = mpc一次周期内必须到达目标速度
  scalar_t z_change = cmdVelRot(2) * timeToTarget;    //高度变化
  if (fabs(cmdVelRot(0)) < 0.06)      //如果速度指令太小，则给0
    cmdVelRot(0) = 0;
  else if (fabs(cmdVelRot(1)) < 0.06)
    cmdVelRot(1) = 0;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;   //目标位置x、y
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;   
    target(2) = COM_HEIGHT;   //目标高度 = 质心高度
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;  //目标yaw
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;    //计算目标时刻
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);    //生成位置参考轨迹
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;    //将给定质心速度填入到参考轨迹的前3位，起点和终点相同，为了加快相应
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

TargetTrajectories cmdPosToTargetTrajectories(const vector_t& cmdPos, const SystemObservation& observation) //给定姿态生成参考轨迹
{
  const vector_t currentPose = observation.state.segment<6>(6);
  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0);     //位置x、y不变
    target(1) = currentPose(1);
    scalar_t dz = COM_HEIGHT - currentPose(2);
    dz = dz > 0 ? fmin(dz, changeLimit_[2]) : fmax(dz, -changeLimit_[2]);
    target(2) = currentPose(2) + dz;    //仍然保持质心高度
    target(3) = cmdPos[0];    //目标姿态 = 给定姿态
    target(4) = cmdPos[1];
    target(5) = cmdPos[2];
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = bodyRotationToTargetTrajectories(targetPose, observation, targetReachingTime);  //生成参考轨迹
  return trajectories;
}

int main(int argc, char** argv)
{
  const std::string robotName = "legged_robot";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");   //初始化legged_robot_target节点，节点已经在one_start.launch创建
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);    //读取设定质心高度
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);   //mpc时间周期

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories,
                                                  &cmdVelToTargetTrajectories, &cmdPosToTargetTrajectories);    //发布话题，生成参考轨迹

  ros::spin();  //开始循环
  return 0;
}
