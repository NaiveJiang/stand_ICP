//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_wbc/WeightedWbc.h"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include "pinocchio/spatial/fwd.hpp"
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <qpOASES.hpp>

namespace legged
{
  vector_t WeightedWbc::update(const vector_t &stateDesired, const vector_t &inputDesired,
                               const vector_t &rbdStateMeasured, size_t mode, scalar_t period, const SystemObservation &observation)
  {
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, observation);

    // Constraints
    Task constraints = formulateConstraints();
    size_t numConstraints = constraints.b_.size() + constraints.f_.size();

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
    vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_; // clang-format on

    // Cost
    Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
        weighedTask.a_.transpose() * weighedTask.a_;
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

    // Solve
    auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);
    int nWsr = 20;

    qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    vector_t qpSol(getNumDecisionVars());

    qpProblem.getPrimalSolution(qpSol.data());

    if (!qpProblem.isSolved())
    {
      std::cout << "ERROR: WeightWBC Not Solved!!!" << std::endl;
      if (last_qpSol.size() > 0)
        qpSol = last_qpSol;
    }

    last_qpSol = qpSol;

    // const auto &model = pinocchioInterfaceMeasuredQuat_.getModel();
    // auto &data = pinocchioInterfaceMeasuredQuat_.getData();

    const auto &model = pinocchioInterfaceMeasured_.getModel();
    auto &data = pinocchioInterfaceMeasured_.getData();

    // // // pinocchio::dccrba(model, data, qMeasuredQ_, vMeasured_);
    // // // pinocchio::computeCentroidalMomentumTimeVariation(model, data);
    // // // pinocchio::centerOfMass(model, data, qMeasuredQ_, vMeasured_);

    // vector3_t rcm = comPosition_; // 质心位置
    // PINOCCHIO_ALIGNED_STD_VECTOR(pinocchio::Force) fext(model.joints.size(), pinocchio::Force::Zero());
    // size_t FRAME_ID = 0;

    // std::vector<vector3_t> r(info_.numThreeDofContacts);
    // vector3_t feetPositionsMeasure;
    // feetPositionsMeasure.setZero();

    // for (size_t i = 0; i < info_.numThreeDofContacts; i++)
    // {
    //   FRAME_ID = info_.endEffectorFrameIndices[i];
    //   feetPositionsMeasure = data.oMf[FRAME_ID].translation();
    //   r[i] = feetPositionsMeasure - rcm;
    //   const vector3_t translationJointFrameToContactFrame = model.frames[FRAME_ID].placement.translation();
    //   const matrix3_t rotationWorldFrameToJointFrame = data.oMi[model.frames[FRAME_ID].parent].rotation().transpose();

    //   fext[model.frames[FRAME_ID].parent].linear() = /*rotationWorldFrameToJointFrame * qpSol.segment(6 + info_.actuatedDofNum + 3 * i, 3)*/ rotationWorldFrameToJointFrame * -data.mass[0] * model.gravity981 / 8;
    //   fext[model.frames[FRAME_ID].parent].angular() = rotationWorldFrameToJointFrame * skewSymmetric(r[i]) * (-data.mass[0] * model.gravity981 / 8);
    //   // fext[model.frames[FRAME_ID].parent].angular().setZero();
    // }
    // // // // // fext[model.getJointId("leg_l6_joint")].linear() = -data.mass[0] * model.gravity981 / 2;
    // // // // // fext[model.getJointId("leg_r6_joint")].linear() = -data.mass[0] * model.gravity981 / 2;
    // auto v1 = vMeasured_;
    // // v1.setZero();
    // auto a1 = qpSol.segment(0, info_.generalizedCoordinatesNum);

    // a1.setZero();
    // a1.segment(0,3) += -model.gravity981;

    // auto q1 = qMeasuredQ_;
    // q1.segment<4>(3) = vector4_t(0, 0, 0, 1);
    // pinocchio::forwardKinematics(model, data, qMeasuredQ_, v1, a1);
    // pinocchio::updateFramePlacements(model, data);
    // auto g1 = pinocchio::computeGeneralizedGravity(model, data, qMeasured_);
    // // auto nle = pinocchio::nonLinearEffects(model, data, qMeasured_, v1);
    // // // a1.segment<3>(0) += -model.gravity981;
    // auto id_torque = pinocchio::rnea(model, data, qMeasuredQ_, v1, a1, fext);
    // // // auto id_torque = pinocchio::rnea(model, data, q1, v1, a1) - g1;
    // // // auto id_torque = pinocchio::rnea(model, data, qMeasured_, vMeasured_, qpSol.segment(0, info_.generalizedCoordinatesNum), fext) - g1;

    // // std::cout << "*************q*************" << std::endl;
    // // std::cout << qMeasuredQ_.segment<4>(3).transpose() << std::endl;

    // std::cout << "*************id*************" << std::endl;
    // std::cout << id_torque.tail(info_.actuatedDofNum) << std::endl;

    // std::cout << "*************qp acc*************" << std::endl;
    // std::cout << a1 << std::endl;
    // 逆动力学
    auto tau = data.M * qpSol.head(6) - j_.transpose() * qpSol.segment(6 + info_.actuatedDofNum, 3 * info_.numThreeDofContacts) + data.nle;
    // std::cout << "*************solver*************" << std::endl;
    // std::cout << tau.tail(info_.actuatedDofNum) << std::endl;

    auto tauoo = qpSol;
    tauoo.tail(info_.actuatedDofNum) = tau.tail(info_.actuatedDofNum);
    return tauoo;
  }

  Task WeightedWbc::formulateConstraints()
  {
    if (stance_mode_){
      return /*formulateFloatingBaseEomTask()*/ formulateCentroidalDynamicsTask() + formulateFrictionConeTask() 
    + formulateBaseAngularMotionTask()
#if USE_6_AXIS_HAND
    + formulateHandTask()
#else 
    + formulateArmTask()
#endif 
    + formulatefootTask();
    }
    else{
      return formulateFloatingBaseEomTask() + formulateFrictionConeTask() + formulateBaseAngularMotionTask();
    }
    
  }

  Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period)
  {
    if (stance_mode_)
      return formulateMomentumTask(period) * 1.0 + formulateJointAccSmallTask() * 0.05 + formulateGRFSmallTask() * 0.01;
    else
      return formulateSwingLegTask() * weightSwingLeg_ +
             formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
             formulateContactForceTask(inputDesired) * weightContactForce_ +
             formulateNoContactMotionTask() * contactMotionScale_ +
             formulateZeroTorque() * zeroTorque_;
  }

  Task WeightedWbc::formulateStanceBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired,
                                                 scalar_t period)
  {
    matrix_t a(6, numDecisionVars_);
    a.setZero();
    a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6); // 令加速度全为0

    vector6_t b;
    b.setZero();

    return {a, b, matrix_t(), vector_t()};
  }

  void WeightedWbc::loadTasksSetting(const std::string &taskFile, bool verbose)
  {
    WbcBase::loadTasksSetting(taskFile, verbose);

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    std::string prefix = "weight.";
    if (verbose)
    {
      std::cerr << "\n #### WBC weight:";
      std::cerr << "\n #### =============================================================================\n";
    }
    loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
    loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
    loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
    loadData::loadPtreeValue(pt, contactMotionScale_, prefix + "contactMotion", verbose);
    loadData::loadPtreeValue(pt, zeroTorque_, prefix + "zeroTorque", verbose);
  }

} // namespace legged