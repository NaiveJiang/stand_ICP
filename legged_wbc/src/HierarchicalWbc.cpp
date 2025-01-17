//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_wbc/HierarchicalWbc.h"

#include "legged_wbc/HoQp.h"

namespace legged
{
vector_t HierarchicalWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                                 const vector_t& rbdStateMeasured, size_t mode, scalar_t period,const SystemObservation &observation)
{
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, observation);
  //混合WBC
  if (stance_mode_){
    Task task0 = formulateCentroidalDynamicsTask() + formulateFrictionConeTask(); // 最高优先级
    Task task1 = formulateMomentumTask(period) ;
    Task task2 = formulateBaseAngularMotionTask() + formulateArmTask() + formulatefootTask();
    Task task3 = formulateJointAccSmallTask() * 0.05 + formulateGRFSmallTask() * 0.01;
    HoQp hoQp(task3, std::make_shared<HoQp>
              (task2, std::make_shared<HoQp>
              (task1, std::make_shared<HoQp>(task0))));
    return hoQp.getSolutions();
  }
  else{
    Task task0 = formulateFloatingBaseEomTask(); // 最高优先级
    Task task1 = formulateNoContactMotionTask() * contactMotionScale_ + formulateTorqueLimitsTask() + formulateFrictionConeTask();
    Task task2 = formulateBaseXYLinearAccelTask() * baseAccel_;
    Task task3 = formulateBaseAngularMotionTask();
    Task task4 = formulateBaseHeightMotionTask();
    Task task5 = formulateSwingLegTask() * swingLeg_ + formulateContactForceTask(inputDesired) * contactForce_;
    Task task6 = formulateZeroTorque() * zeroTorque_;
    HoQp hoQp(task6, std::make_shared<HoQp>(task5, std::make_shared<HoQp>(task4, std::make_shared<HoQp>(task3, std::make_shared<HoQp>(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)))))));
    return hoQp.getSolutions();
  }
  
  // std::cout << "HierarchicalWbc running" << std::endl;
  
}

void HierarchicalWbc::loadTasksSetting(const std::string &taskFile, bool verbose)
{
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "hierarchical.";
  if (verbose)
  {
    std::cerr << "\n #### WBC hierarchical:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, baseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, contactForce_, prefix + "contactForce", verbose);
  loadData::loadPtreeValue(pt, zeroTorque_, prefix + "zeroTorque", verbose);
  loadData::loadPtreeValue(pt, contactMotionScale_, prefix + "contactMotion", verbose);
}

//站立任务，可能实物需要用？
Task HierarchicalWbc::formulateStanceBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired,
                                                   scalar_t period)
{
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  vector6_t b;
  b.setZero();

  return {a, b, matrix_t(), vector_t()};
}

//零力矩任务，放最后一级
Task HierarchicalWbc::formulateZeroTorque()
{
  matrix_t a(info_.actuatedDofNum, numDecisionVars_);
  a.setZero();
  a.block(0, numDecisionVars_ - info_.actuatedDofNum, info_.actuatedDofNum, info_.actuatedDofNum) = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);

  vector6_t b;
  b.setZero();
  
  return {a, b, matrix_t(), vector_t()};
}

}  // namespace legged
