//
// Created by qiayuan on 2022/7/1.
//

// some ref: https://github.com/skywoodsz/qm_control

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include "legged_wbc/WbcBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>
#include <pinocchio/algorithm/center-of-mass.hpp>

namespace legged
{
  WbcBase::WbcBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                   const PinocchioEndEffectorKinematics &eeKinematics, const PinocchioInterface &pinocchioInterfaceQuat)
      : pinocchioInterfaceMeasured_(pinocchioInterface), pinocchioInterfaceDesired_(pinocchioInterface), pinocchioInterfaceMeasuredQuat_(pinocchioInterfaceQuat), info_(std::move(info)), mapping_(info_), inputLast_(vector_t::Zero(info_.inputDim)), eeKinematics_(eeKinematics.clone()), rbdConversions_(pinocchioInterface, info_)
  {
    // linear force, plus angular force
    contact_force_size_ = 3 * info_.numThreeDofContacts;
    numDecisionVars_ = info_.generalizedCoordinatesNum + contact_force_size_ + info_.actuatedDofNum; // 18+3*4+12=54
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    // //test
    qMeasuredQ_ = vector_t(pinocchioInterfaceQuat.getModel().nq);
    qMeasuredQ_ = vector_t(pinocchioInterfaceQuat.getModel().nq);

    cmd_body_pos_.resize(6);
    cmd_body_pos_.setZero();
    cmd_body_vel_.resize(6);
    cmd_body_vel_.setZero();
    earlyLatecontact_[0].fill(false);
    earlyLatecontact_[1].fill(false);

    // 估计的足端力
    feetForcesMeasure.reserve(info_.numThreeDofContacts);
    feetTorqueMeasure.reserve(info_.numThreeDofContacts);
    feetForcesMeasure.clear();
    feetTorqueMeasure.clear();

    dcmDesired_.setZero();
    dcmVelDesired_.setZero();
    dcmMeasured_.setZero();
    dcmVelMeasured_.setZero();

    comPosition_.setZero();
    comVelocity_.setZero();

    footPosDes_.setZero();
    footPosMea_.setZero();

#if USE_6_AXIS_FOOT
    footVelDes_.resize(12);
    footVelMea_.resize(12);
    footRotDes_ = matrix_t(6, 3);
    footRotMea_ = matrix_t(6, 3);
    footRotDes_.setZero();
    footRotMea_.setZero();
#endif
    footVelDes_.setZero();
    footVelMea_.setZero();

    armPosDesired_.resize(14);
    armPosDesired_.setZero();
    armVelDesired_.resize(14);
    armVelDesired_.setZero();

#if USE_6_AXIS_HAND
    handPosDes_.setZero();
    handPosMea_.setZero();
    handVelDes_.resize(12);
    handVelMea_.resize(12);
    handRotDes_ = matrix_t(6, 3);
    handRotMea_ = matrix_t(6, 3);
    handRotDes_.setZero();
    handRotMea_.setZero();
    handVelDes_.setZero();
    handVelMea_.setZero();
#endif

    Ag(6, info_.generalizedCoordinatesNum);
    dAg(6, info_.generalizedCoordinatesNum);
    Ag.setZero();
    dAg.setZero();

    hg.setZero();
    dhg.setZero();

    i_ric.setZero();
    copPostion.setZero();

    dcmDesired_.z() = 0.8;
    omega0 = sqrt(-pinocchioInterfaceMeasured_.getModel().gravity981.z() / dcmDesired_.z());
    pinocchio::computeTotalMass(pinocchioInterfaceMeasured_.getModel(), pinocchioInterfaceMeasured_.getData());
    mass_ = pinocchioInterfaceMeasured_.getData().mass[0];
  }

  vector_t WbcBase::update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                           size_t mode, scalar_t /*period*/, const SystemObservation & observation)
  {
    contactFlag_ = modeNumber2StanceLeg(mode);
    numContacts_ = 0;
    for (bool flag : contactFlag_) // 计算接触地面的点的个数
    {
      if (flag)
      {
        numContacts_++;
      }
    }

    updateMeasured(rbdStateMeasured, observation);
    updateDesired(stateDesired, inputDesired);

    return {};
  }

  void WbcBase::updateMeasured(const vector_t &rbdStateMeasured, const SystemObservation &observation)
  {
    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);                                     // xyz linaer pos  来自机体估计出来的位置
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();                                     // 来自机体估计出来的姿态
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum); // 编码器的关节角度
    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);   // 估计的机体速度
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
        qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum)); // 估计的机体角速度(角度微分)
    vMeasured_.tail(info_.actuatedDofNum) =
        rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

    // test
    qMeasuredQ_.head<3>() = rbdStateMeasured.segment<3>(3);                                                                  // xyz linaer pos  来自机体估计出来的位置
    qMeasuredQ_.segment<4>(3) = getQuaternionFromEulerAnglesZyx<scalar_t>(rbdStateMeasured.head<3>()).normalized().coeffs(); // 来自机体估计出来的姿态
    qMeasuredQ_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);                              // 编码器的关节角度

    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();

    const auto &modelq = pinocchioInterfaceMeasuredQuat_.getModel();
    auto &dataq = pinocchioInterfaceMeasuredQuat_.getData();

    // For floating base EoM task
    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data, qMeasured_);
    pinocchio::updateFramePlacements(model, data);
    // test
    pinocchio::forwardKinematics(modelq, dataq, qMeasuredQ_, vMeasured_);
    pinocchio::updateFramePlacements(modelq, dataq);

    pinocchio::crba(model, data, qMeasured_);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
      j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
    dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
      jac.setZero(6, info_.generalizedCoordinatesNum);
      pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                               pinocchio::LOCAL_WORLD_ALIGNED, jac);
      dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    // For base motion tracking task
    base_j_.setZero(6, info_.generalizedCoordinatesNum);
    base_dj_.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId("base_link"), pinocchio::LOCAL_WORLD_ALIGNED, base_j_);
    pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base_link"), pinocchio::LOCAL_WORLD_ALIGNED,
                                             base_dj_);
    
#if USE_6_AXIS_FOOT
    jf = matrix_t(12, info_.generalizedCoordinatesNum).setZero(); // 脚板雅可比
    djf = matrix_t(12, info_.generalizedCoordinatesNum).setZero();
    auto frameID = model.getFrameId("leg_l6_link", pinocchio::BODY);
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jf0;
    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    jf.block(0, 0, 6, info_.generalizedCoordinatesNum) = jf0;

    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    djf.block(0, 0, 6, info_.generalizedCoordinatesNum) = jf0;

    // 脚板的位置和速度
    footPosMea_.segment<3>(0) = data.oMf[frameID].translation();
    footRotMea_.block(0, 0, 3, 3) = data.oMf[frameID].rotation();
    footVelMea_.segment<6>(0) = jf * vMeasured_;

    frameID = model.getFrameId("leg_r6_link", pinocchio::BODY);
    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    jf.block(6, 0, 6, info_.generalizedCoordinatesNum) = jf0;

    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    djf.block(6, 0, 6, info_.generalizedCoordinatesNum) = jf0;

    // 脚板的位置和速度
    footPosMea_.segment<3>(3) = data.oMf[frameID].translation();
    footRotMea_.block(3, 0, 3, 3) = data.oMf[frameID].rotation();
    footVelMea_.segment<6>(6) = jf * vMeasured_;
#else
    jf = matrix_t(6, info_.generalizedCoordinatesNum).setZero(); // 脚板雅可比
    djf = matrix_t(6, info_.generalizedCoordinatesNum).setZero();
    auto frameID = model.getFrameId("leg_l6_link", pinocchio::BODY);
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jf0;
    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    jf.block(0, 0, 3, info_.generalizedCoordinatesNum) = jf0.topRows<3>();

    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    djf.block(0, 0, 3, info_.generalizedCoordinatesNum) = jf0.topRows<3>();

    // 脚板的位置和速度
    footPosMea_.segment<3>(0) = data.oMf[frameID].translation();
    footVelMea_.segment<3>(0) = jf.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    frameID = model.getFrameId("leg_r6_link", pinocchio::BODY);
    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    jf.block(3, 0, 3, info_.generalizedCoordinatesNum) = jf0.topRows<3>();

    jf0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf0);
    djf.block(3, 0, 3, info_.generalizedCoordinatesNum) = jf0.topRows<3>();

    // 脚板的位置和速度
    footPosMea_.segment<3>(3) = data.oMf[frameID].translation();
    footVelMea_.segment<3>(3) = jf.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
#endif

#if USE_6_AXIS_HAND
    jh = matrix_t(12, info_.generalizedCoordinatesNum).setZero(); // 手雅可比
    djh = matrix_t(12, info_.generalizedCoordinatesNum).setZero();

    frameID = model.getFrameId("zarm_l_f1_link", pinocchio::BODY);
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jh0;
    jh0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jh0);
    jh.block(0, 0, 6, info_.generalizedCoordinatesNum) = jh0;

    jh0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jh0);
    djh.block(0, 0, 6, info_.generalizedCoordinatesNum) = jh0;

    // 手的位置和速度
    handPosMea_.segment<3>(0) = data.oMf[frameID].translation() - data.oMf[model.getBodyId("base_link")].translation();
    handRotMea_.block(0, 0, 3, 3) = data.oMf[frameID].rotation();
    handVelMea_.segment<6>(0) = jh * vMeasured_;

    frameID = model.getFrameId("zarm_r_f1_link", pinocchio::BODY);
    jh0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jh0);
    jh.block(6, 0, 6, info_.generalizedCoordinatesNum) = jh0;

    jh0.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jh0);
    djh.block(6, 0, 6, info_.generalizedCoordinatesNum) = jh0;

    // 手的位置和速度
    handPosMea_.segment<3>(3) = data.oMf[frameID].translation() - data.oMf[model.getBodyId("base_link")].translation(); // 要在局部坐标系下
    handRotMea_.block(3, 0, 3, 3) = data.oMf[frameID].rotation();
    handVelMea_.segment<6>(6) = jh * vMeasured_;
#endif

    eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> feetPositionsMeasure = eeKinematics_->getPosition(observation.state);
    double sum_z = 0.0;
    scalar_t avg_feetPos_x = 0.0;
    scalar_t avg_feetPos_y = 0.0;
    for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    {
      feetForcesMeasure[i] = centroidal_model::getContactForces(observation.input, i, info_);
      sum_z += feetForcesMeasure[i].z();
      copPostion += feetForcesMeasure[i].z() * feetPositionsMeasure[i];
      avg_feetPos_x += feetPositionsMeasure[i].x();
      avg_feetPos_y += feetPositionsMeasure[i].y();
    }
    if (sum_z > 0)
      copPostion /= sum_z;

    pinocchio::dccrba(model, data, qMeasured_, vMeasured_);
    pinocchio::computeCentroidalMomentumTimeVariation(model, data);
    pinocchio::centerOfMass(model, data, qMeasured_, vMeasured_);

    Ag = data.Ag;
    dAg = data.dAg;

    hg = data.hg;
    dhg = data.dhg;

    comPosition_ = data.com[0];
    comVelocity_ = data.vcom[0];

    dcmMeasured_ = comPosition_ + comVelocity_ / omega0;
    if (!init_flg)
    {
      dcmDesired_.x() = comPosition_.x();
      dcmDesired_.y() = comPosition_.y();
      dcmDesired_.z() = comPosition_.z();
    }
    else{
      // dcmDesired_.z() = comPosition_.z();
      dcmDesired_.x() = -(footPosMea_(0) + footPosMea_(3)) * 0.5;
      dcmDesired_.y() = -(footPosMea_(1) + footPosMea_(4)) * 0.5;
      omega0 = sqrt(-model.gravity981.z() / dcmDesired_.z());
    }
    // std::cout << avg_feetPos_x / info_.numThreeDofContacts << std::endl;
    // std::cout << avg_feetPos_y / info_.numThreeDofContacts << std::endl;
  }

  void WbcBase::updateDesired(const vector_t &stateDesired, const vector_t &inputDesired) // 输入MPC的状态
  {
    const auto &model = pinocchioInterfaceDesired_.getModel(); // 只读
    auto &data = pinocchioInterfaceDesired_.getData();         // 可以修改

    mapping_.setPinocchioInterface(pinocchioInterfaceDesired_);
    const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired); // 期望的关节角度

    armPosDesired_ = mapping_.getPinocchioJointPosition(stateDesired).tail(14);
    pinocchio::forwardKinematics(model, data, qDesired);
    pinocchio::computeJointJacobians(model, data, qDesired); // 计算所有关节的雅可比
    pinocchio::updateFramePlacements(model, data);           // 更新所有frame的位置
    updateCentroidalDynamics(pinocchioInterfaceDesired_, info_, qDesired);
    const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

    armVelDesired_ = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired).tail(14);
    pinocchio::forwardKinematics(model, data, qDesired, vDesired);

#if USE_6_AXIS_FOOT
    auto frameID = model.getFrameId("leg_l6_link", pinocchio::BODY);
    if (!init_flg){
      footPosDes_.segment<3>(0) = data.oMf[frameID].translation();
      footRotDes_.block(0, 0, 3, 3) = data.oMf[frameID].rotation();
    }

    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jf_des;
    jf_des.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf_des);
    footVelDes_.segment<6>(0) = jf_des * vDesired;

    frameID = model.getFrameId("leg_r6_link", pinocchio::BODY);
    if (!init_flg){
      footPosDes_.segment<3>(3) = data.oMf[frameID].translation();
      footRotDes_.block(3, 0, 3, 3) = data.oMf[frameID].rotation();
    }

    jf_des.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf_des);
    footVelDes_.segment<6>(6) = jf_des * vDesired;
#else
    auto frameID = model.getFrameId("leg_l6_link", pinocchio::BODY);
    footPosDes_.segment<3>(0) = data.oMf[frameID].translation();
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jf_des;
    jf_des.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf_des);
    footVelDes_.segment<3>(0) = jf_des.block(0, 0, 3, info_.generalizedCoordinatesNum) * vDesired;

    frameID = model.getFrameId("leg_r6_link", pinocchio::BODY);
    footPosDes_.segment<3>(3) = data.oMf[frameID].translation();
    jf_des.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jf_des);
    footVelDes_.segment<3>(3) = jf_des.block(3, 0, 3, info_.generalizedCoordinatesNum) * vDesired;
#endif

#if USE_6_AXIS_HAND
    frameID = model.getFrameId("zarm_l_f1_link", pinocchio::BODY);
    if (!init_flg){
      handPosDes_.segment<3>(0) = data.oMf[frameID].translation() - data.oMf[model.getBodyId("base_link")].translation();
      handRotDes_.block(0, 0, 3, 3) = data.oMf[frameID].rotation();
    }
    
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jh_des;
    jh_des.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jh_des);
    handVelDes_.segment<6>(0) = jh_des * vDesired;

    frameID = model.getFrameId("zarm_r_f1_link", pinocchio::BODY);
    if (!init_flg){
      handPosDes_.segment<3>(3) = data.oMf[frameID].translation() - data.oMf[model.getBodyId("base_link")].translation();
      handRotDes_.block(3, 0, 3, 3) = data.oMf[frameID].rotation();
    }

    jh_des.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, frameID, pinocchio::LOCAL_WORLD_ALIGNED, jh_des);
    handVelDes_.segment<6>(6) = jh_des * vDesired;
#endif

    const vector_t jointAccelerations = vector_t::Zero(info_.actuatedDofNum); // 保存关节加速度
    rbdConversions_.computeBaseKinematicsFromCentroidalModel(stateDesired, inputDesired, jointAccelerations, basePoseDes_,
                                                             baseVelocityDes_, baseAccelerationDes_); // 得到base的加速度期望
  }

  Task WbcBase::formulateFloatingBaseEomTask()
  {
    auto &data = pinocchioInterfaceMeasured_.getData();
    matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    s.block(0, 0, info_.actuatedDofNum, 6).setZero();
    s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
    matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose())
                     .finished();
    vector_t b = -data.nle;

    return {a, b, matrix_t(), vector_t()}; // 第一个是等式约束的A，第二个是等式约束的b；第三个是不等式约束的D，第四个是不等式约束的f
  }

  Task WbcBase::formulateTorqueLimitsTask()
  {
    matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
    d.setZero();
    matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
    d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
            info_.actuatedDofNum) = i;
    d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
            info_.actuatedDofNum) = -i;
    vector_t f(2 * info_.actuatedDofNum);
    const int dofPerLeg = info_.actuatedDofNum / 2;
    for (size_t l = 0; l < 2 * info_.actuatedDofNum / dofPerLeg; ++l)
    {
      f.segment(dofPerLeg * l, dofPerLeg) = torqueLimits_;
    }
    return {matrix_t(), vector_t(), d, f};
  }

  Task WbcBase::formulateNoContactMotionTask()
  {
    matrix_t a(3 * numContacts_, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    {
      if (contactFlag_[i])
      {
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateFrictionConeTask()
  {
    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    a.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (!contactFlag_[i])
      {
        a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3); // 无接触时没有反作用力
      }
    }
    vector_t b(a.rows());
    b.setZero();

    matrix_t frictionPyramic(6, 3); // clang-format off
    frictionPyramic << 0, 0, -1,                //-fz
                     0, 0, 1,                   //fz
                     1, 0, -frictionCoeff_,   //fx-ufz
                    -1, 0, -frictionCoeff_,   //-fx-ufz
                     0, 1, -frictionCoeff_,   //fy-ufz
                     0,-1, -frictionCoeff_; // clang-format on

    matrix_t d(6 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);  //接触的加入摩擦锥  没接触的不约束
    d.setZero();
    vector_t f = Eigen::VectorXd::Zero(d.rows());
    j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (contactFlag_[i])
      {
        d.block(6 * j, info_.generalizedCoordinatesNum + 3 * i, 6, 3) = frictionPyramic;
        f.segment(6 * j, 2) << -10, 1000;
        j++;
      }
    }

    return {a, b, d, f};
  }

  // Tracking base xy linear motion task
  Task WbcBase::formulateBaseXYLinearAccelTask()
  {
    matrix_t a(2, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 2, 2) = matrix_t::Identity(2, 2);
    b = baseAccelerationDes_.segment<2>(0);

    return {a, b, matrix_t(), vector_t()};
  }

  // test
  Task WbcBase::formulateBaseXYLinearAccelTaskInStance(scalar_t period)
  {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 3, 3) = matrix_t::Identity(3, 3);

    // 要改basePoseDes_ qMeasured_和vMeasured_  qMeasured_从编码器获取机体的位置   vMeasured_估计出来的速度
    const auto &modelMeasured = pinocchioInterfaceMeasured_.getModel();
    auto &dataMeasured = pinocchioInterfaceMeasured_.getData();

    const auto &modelDesired = pinocchioInterfaceDesired_.getModel();
    auto &dataDesired = pinocchioInterfaceDesired_.getData();

    auto base_idMeasured = modelMeasured.getBodyId("base_link");
    auto base_idDesired = modelDesired.getBodyId("base_link");

    feet_array_t<vector3_t> foot_pos_in_worldDesired;
    feet_array_t<vector3_t> foot_pos_in_worldMeasured;

    vector3_t base_pos_in_worldDesired = dataDesired.oMf[base_idDesired].translation();
    vector3_t base_pos_in_worldMeasured = dataMeasured.oMf[base_idMeasured].translation();

    vector3_t base_pos_in_footMeasured;
    vector3_t base_pos_in_footDesired;

    for (int leg = 0; leg < foot_pos_in_worldMeasured.size(); leg++)
    {
      auto FRAME_ID = info_.endEffectorFrameIndices[leg];
      foot_pos_in_worldMeasured[leg] = dataMeasured.oMf[FRAME_ID].translation(); // 足端的帧在世界坐标的变换矩阵，translation得到变换矩阵的位置部分
      foot_pos_in_worldDesired[leg] = dataDesired.oMf[FRAME_ID].translation();

      // 计算base和足端的相对位置
      base_pos_in_footMeasured = dataMeasured.oMf[FRAME_ID].rotation().transpose() * (base_pos_in_worldMeasured - foot_pos_in_worldMeasured[leg]);
      base_pos_in_footDesired = dataDesired.oMf[FRAME_ID].rotation().transpose() * (base_pos_in_worldDesired - foot_pos_in_worldDesired[leg]);
    }

    b = baseAccelerationDes_.segment<3>(0) + basexyKp_ * (base_pos_in_footDesired - base_pos_in_footMeasured) +
        basexyKd_ * (baseVelocityDes_.segment<3>(0) - vMeasured_);

    return {a, b, matrix_t(), vector_t()};
  }

  // Tracking base height motion task
  Task WbcBase::formulateBaseHeightMotionTask()
  {
    matrix_t a(1, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 2, 1, 1).setIdentity();

    b[0] = baseAccelerationDes_[2] + baseHeightKp_ * (/*basePoseDes_[2]*/ 0.8 - qMeasured_[2]) +
           baseHeightKd_ * (baseVelocityDes_[2] - vMeasured_[2]);

    return {a, b, matrix_t(), vector_t()};
  }

  // Tracking base angular motion task
  Task WbcBase::formulateBaseAngularMotionTask()
  {
    matrix_t a(3, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 3, info_.generalizedCoordinatesNum) = base_j_.block(3, 0, 3, info_.generalizedCoordinatesNum);

    vector3_t eulerAngles = qMeasured_.segment<3>(3); // z y x  yaw  pitch roll

    // from derivative euler to angular
    vector3_t vMeasuredGlobal =
        getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngles, vMeasured_.segment<3>(3));
    vector3_t vDesiredGlobal = baseVelocityDes_.tail<3>();

    // from euler to rotation
    vector3_t eulerAnglesDesired = basePoseDes_.tail<3>();
    matrix3_t rotationBaseMeasuredToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAngles);
    matrix3_t rotationBaseReferenceToWorld = getRotationMatrixFromZyxEulerAngles<scalar_t>(eulerAnglesDesired);

    vector3_t error = rotationErrorInWorld<scalar_t>(rotationBaseReferenceToWorld, rotationBaseMeasuredToWorld);

    // desired acc
    vector3_t accDesired = baseAccelerationDes_.tail<3>();

    b = accDesired + baseAngularKp_ * error + baseAngularKd_ * (vDesiredGlobal - vMeasuredGlobal) -
        base_dj_.block(3, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
  {
    return formulateBaseXYLinearAccelTask() + formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask();
  }

  Task WbcBase::formulateSwingLegTask()
  {
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
    std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
    eeKinematics_->setPinocchioInterface(pinocchioInterfaceDesired_);
    std::vector<vector3_t> posDesired = eeKinematics_->getPosition(vector_t());
    std::vector<vector3_t> velDesired = eeKinematics_->getVelocity(vector_t(), vector_t());

    matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();
    b.setZero();
    size_t j = 0;
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      if (!contactFlag_[i])
      {
        vector3_t accel = swingKp_ * (posDesired[i] - posMeasured[i]) + swingKd_ * (velDesired[i] - velMeasured[i]);
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
        j++;
      }
    }

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateContactForceTask(const vector_t &inputDesired) const
  {
    matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
    vector_t b(a.rows());
    a.setZero();

    for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
    {
      a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
    b = inputDesired.head(a.rows());

    return {a, b, matrix_t(), vector_t()};
  }

  // 零力矩任务，放最后一级
  Task WbcBase::formulateZeroTorque()
  {
    matrix_t a(info_.actuatedDofNum, numDecisionVars_);
    a.setZero();
    a.block(0, numDecisionVars_ - info_.actuatedDofNum, info_.actuatedDofNum, info_.actuatedDofNum) = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

    vector6_t b;
    b.setZero();

    return {a, b, matrix_t(), vector_t()};
  }

  void WbcBase::compensateFriction(vector_t &x)
  {
    vector_t coulomb_friction(info_.actuatedDofNum);
    vector_t joint_v = vMeasured_.tail(info_.actuatedDofNum);
    for (int i = 0; i < info_.actuatedDofNum; i++)
    {
      const int sgn = (joint_v[i] > 0) - (joint_v[i] < 0);
      coulomb_friction[i] = (abs(joint_v[i]) > 0.001) ? (sgn * 0.2) : 0;
    }
    x.tail(12) = x.tail(12) + coulomb_friction;
  }

  // test
  vector6_t WbcBase::getGroundWrench(const SystemObservation &observation)
  {
    vector6_t grWrench; // 地面反作用力旋量
    grWrench.setZero();

    const auto &model_mea = pinocchioInterfaceMeasured_.getModel();
    auto &data_mea = pinocchioInterfaceMeasured_.getData();

    pinocchio::ccrba(model_mea, data_mea, qMeasured_, vMeasured_);
    pinocchio::computeCentroidalMomentumTimeVariation(model_mea, data_mea);
    pinocchio::centerOfMass(model_mea, data_mea, qMeasured_, vMeasured_);

    auto rcm = data_mea.com[0]; // 质心位置

    std::vector<vector3_t> r(info_.numThreeDofContacts);
    std::vector<vector3_t> feetPositionsMeasure = eeKinematics_->getPosition(observation.state);
    for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    {
      feetForcesMeasure[i] = centroidal_model::getContactForces(observation.input, i, info_);
      r[i] = feetPositionsMeasure[i] - rcm;
      feetTorqueMeasure[i] = skewSymmetric(r[i]) * feetForcesMeasure[i];

      // pinocchio的力旋量 [力 力矩]
      grWrench.segment(0, 3) += feetForcesMeasure[i];
      grWrench.segment(3, 3) += feetTorqueMeasure[i];
    }
    return grWrench;
  }

  Task WbcBase::formulateMomentumTask(scalar_t period)
  {
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    vector_t h_des(a.rows());
    h_des.setZero();

    auto dcm_bias = dcmMeasured_ - dcmDesired_;
    auto rcm_z_bias = dcmDesired_(2) - comPosition_(2);

    if (init_flg)
    {
      i_ric += dcm_bias * period;
    }
    h_des.segment(0, 2) = mass_ * omega0 * omega0 * (momLKp_ * dcm_bias.segment<2>(0) + momLKi_ * i_ric.segment<2>(0)) - mass_ * omega0 * comVelocity_.segment<2>(0);

    // h_des.segment(0, 2) = mass_ * (momLKp_ * (comPosition_ - dcmDesired_).segment<2>(0) + momLKd_ * (hg.segment<2>(0) / mass_));
    h_des(2) = mass_ * (momLKzp_ * rcm_z_bias - momLKzd_ * (hg(2) / mass_));
    // scalar_t tau_y = (dcmMeasured_(0) + momLKp_ * dcm_bias(0) + momLKi_ * i_ric(0) - copPostion(0)) * mass_ * pinocchioInterfaceMeasured_.getModel().gravity981.z();
    // scalar_t tau_x = (dcmMeasured_(1) + momLKp_ * dcm_bias(1) + momLKi_ * i_ric(1) - copPostion(1)) * mass_ * pinocchioInterfaceMeasured_.getModel().gravity981.z();
    // h_des(3) = tau_y;
    // h_des(4) = tau_x;
    // h_des(5) = -momAKp_ * hg(5);
    h_des.segment(3, 3) = -momAKp_ * hg.segment<3>(3);
    // h_des.segment(3, 3).setZero();   //使用这个弹性大
    // linearMomentumDesired.segment(0,2) = h_des.segment<2>(0);
    // linearMomentumDesired.segment(3, 1).setZero();
    auto dAdq = dAg * vMeasured_;

    a.block(0, 0, 6, info_.generalizedCoordinatesNum) = Ag.block(0, 0, 6, info_.generalizedCoordinatesNum);
    b = h_des - dAdq.segment<6>(0);

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateCentroidalDynamicsTask()
  {
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    // 重力旋量
    vector6_t Wg;
    Wg.setZero();
    Wg.segment<3>(0) = mass_ * pinocchioInterfaceMeasured_.getModel().gravity981;
    // 接触力旋量
    std::vector<vector3_t> r(info_.numThreeDofContacts);
    size_t FRAME_ID = 0;
    vector3_t feetPositionsMeasure;
    feetPositionsMeasure.setZero();
    matrix_t Q(6, 3 * info_.numThreeDofContacts);
    Q.setZero();
    for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    {
      FRAME_ID = info_.endEffectorFrameIndices[i];
      feetPositionsMeasure = pinocchioInterfaceMeasured_.getData().oMf[FRAME_ID].translation();
      r[i] = feetPositionsMeasure - comPosition_;
      Q.block(0, 3 * i, 3, 3) = matrix_t::Identity(3, 3);
      Q.block(3, 3 * i, 3, 3) = skewSymmetric(r[i]);
    }

    auto dAdq = dAg * vMeasured_;

    a.block(0, 0, 6, info_.generalizedCoordinatesNum) = Ag;
    a.block(0, info_.generalizedCoordinatesNum, 6, 3 * info_.numThreeDofContacts) = -Q;
    b = Wg - dAdq;

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateBaseHightAccTask()
  {
    matrix_t a(1, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 1, info_.generalizedCoordinatesNum) = base_j_.block(2, 0, 1, info_.generalizedCoordinatesNum);
    auto djv = base_dj_.block(0, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
    b[0] = baseAccelerationDes_[2] + baseHeightKp_ * (basePoseDes_[2] - qMeasured_[2]) +
           baseHeightKd_ * (baseVelocityDes_[2] - vMeasured_[2]) - djv[2];

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateJointAccSmallTask()
  {
    matrix_t a(info_.generalizedCoordinatesNum, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    a.block(0, 0, info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum) = matrix_t::Identity(info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum);
    b.setZero();

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateGRFSmallTask()
  {
    matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();
    a.block(0, 0, 3 * info_.numThreeDofContacts, 3 * info_.numThreeDofContacts) = matrix_t::Identity(3 * info_.numThreeDofContacts, 3 * info_.numThreeDofContacts);

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulateArmTask()
  {
    matrix_t a(14, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, info_.generalizedCoordinatesNum - 14, 14, 14).setIdentity();
    b = armKp_ * (armPosDesired_ - qMeasured_.tail(14)) + armKd_ * (armVelDesired_ - vMeasured_.tail(14));

    return {a, b, matrix_t(), vector_t()};
  }

  Task WbcBase::formulatefootTask()
  {
#if USE_6_AXIS_FOOT
    matrix_t a(12, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 12, info_.generalizedCoordinatesNum) = jf;
    vector3_t rotErrL = rotationErrorInWorld<scalar_t>(footRotDes_.block(0, 0, 3, 3), footRotMea_.block(0, 0, 3, 3));
    vector3_t rotErrR = rotationErrorInWorld<scalar_t>(footRotDes_.block(3, 0, 3, 3), footRotMea_.block(3, 0, 3, 3));
    b.segment<3>(0) = footLKp_ * (footPosDes_ - footPosMea_).segment<3>(0) + footLKd_ * (footVelDes_ - footVelMea_).segment<3>(0);
    b.segment<3>(3) = footAKp_ * rotErrL + footAKd_ * (footVelDes_ - footVelMea_).segment<3>(3);
    b.segment<3>(6) = footLKp_ * (footPosDes_ - footPosMea_).segment<3>(3) + footLKd_ * (footVelDes_ - footVelMea_).segment<3>(6);
    b.segment<3>(9) = footAKp_ * rotErrR + footAKd_ * (footVelDes_ - footVelMea_).segment<3>(9);
    b -= djf * vMeasured_;
#else
    matrix_t a(6, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 6, info_.generalizedCoordinatesNum) = jf;
    b = footLKp_ * (footPosDes_ - footPosMea_) + footLKd_ * (footVelDes_ - footVelMea_) - djf * vMeasured_;
#endif

    return {a, b, matrix_t(), vector_t()};
  }

#if USE_6_AXIS_HAND
  Task WbcBase::formulateHandTask()
  {
    matrix_t a(12, numDecisionVars_);
    vector_t b(a.rows());

    a.setZero();
    b.setZero();

    a.block(0, 0, 12, info_.generalizedCoordinatesNum) = jh;
    vector3_t rotErrL = rotationErrorInWorld<scalar_t>(handRotDes_.block(0, 0, 3, 3), handRotMea_.block(0, 0, 3, 3));
    vector3_t rotErrR = rotationErrorInWorld<scalar_t>(handRotDes_.block(3, 0, 3, 3), handRotMea_.block(3, 0, 3, 3));
    b.segment<3>(0) = handLKp_ * (handPosDes_ - handPosMea_).segment<3>(0) + handLKd_ * (handVelDes_ - handVelMea_).segment<3>(0);
    b.segment<3>(3) = handAKp_ * rotErrL + handAKd_ * (handVelDes_ - handVelMea_).segment<3>(3);
    b.segment<3>(6) = handLKp_ * (handPosDes_ - handPosMea_).segment<3>(3) + handLKd_ * (handVelDes_ - handVelMea_).segment<3>(6);
    b.segment<3>(9) = handAKp_ * rotErrR + handAKd_ * (handVelDes_ - handVelMea_).segment<3>(9);
    b -= djh * vMeasured_;

    return {a, b, matrix_t(), vector_t()};
  }
#endif

  void WbcBase::loadTasksSetting(const std::string &taskFile, bool verbose)
  {
    // Load task file
    torqueLimits_ = vector_t(info_.actuatedDofNum / 2);
    loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);
    if (verbose)
    {
      std::cerr << "\n #### Torque Limits Task:";
      std::cerr << "\n #### =============================================================================\n";
      std::cerr << "\n #### motor1, motor2, motor3, motor4, motor5: " << torqueLimits_.transpose() << "\n";
      std::cerr << " #### =============================================================================\n";
    }
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "frictionConeTask.";
    if (verbose)
    {
      std::cerr << "\n #### Friction Cone Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
    if (verbose)
    {
      std::cerr << " #### =============================================================================\n";
    }
    prefix = "swingLegTask.";
    if (verbose)
    {
      std::cerr << "\n #### Swing Leg Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);

    prefix = "baseAccelTask.";
    if (verbose)
    {
      std::cerr << "\n #### Base Accel(Tracking) Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, com_kp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, com_kd_, prefix + "kd", verbose);

    prefix = "baseHeightTask.";
    if (verbose)
    {
      std::cerr << "\n #### Base Height Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, baseHeightKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, baseHeightKd_, prefix + "kd", verbose);
    prefix = "baseAngularTask.";
    if (verbose)
    {
      std::cerr << "\n #### Base Angular Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, baseAngularKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, baseAngularKd_, prefix + "kd", verbose);
    prefix = "baseXYstanceTask.";
    if (verbose)
    {
      std::cerr << "\n #### Base xy stance Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, basexyKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, basexyKd_, prefix + "kd", verbose);
    prefix = "MomentumTask.";
    if (verbose)
    {
      std::cerr << "\n #### Momentum Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, momLKp_, prefix + "Lkp", verbose);
    loadData::loadPtreeValue(pt, momLKd_, prefix + "Lkd", verbose);
    loadData::loadPtreeValue(pt, momLKi_, prefix + "Lki", verbose);
    loadData::loadPtreeValue(pt, momLKzp_, prefix + "Lkzp", verbose);
    loadData::loadPtreeValue(pt, momLKzd_, prefix + "Lkzd", verbose);
    loadData::loadPtreeValue(pt, momAKp_, prefix + "Akp", verbose);
    loadData::loadPtreeValue(pt, momAKd_, prefix + "Akd", verbose);
    prefix = "footTask.";
    if (verbose)
    {
      std::cerr << "\n #### footTask Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, footLKp_, prefix + "Lkp", verbose);
    loadData::loadPtreeValue(pt, footLKd_, prefix + "Lkd", verbose);
#if USE_6_AXIS_FOOT
    loadData::loadPtreeValue(pt, footLKp_, prefix + "Akp", verbose);
    loadData::loadPtreeValue(pt, footLKd_, prefix + "Akd", verbose);
#endif
    prefix = "armTask.";
    if (verbose)
    {
      std::cerr << "\n #### armTask Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, armKp_, prefix + "kp", verbose);
    loadData::loadPtreeValue(pt, armKd_, prefix + "kd", verbose);
#if USE_6_AXIS_HAND
    prefix = "handTask.";
    if (verbose)
    {
      std::cerr << "\n #### handTask Task:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, footLKp_, prefix + "Lkp", verbose);
    loadData::loadPtreeValue(pt, footLKd_, prefix + "Lkd", verbose);
    loadData::loadPtreeValue(pt, footLKp_, prefix + "Akp", verbose);
    loadData::loadPtreeValue(pt, footLKd_, prefix + "Akd", verbose);
#endif
  }

} // namespace legged
