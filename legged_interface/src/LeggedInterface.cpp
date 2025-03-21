//
// Created by qiayuan on 2022/7/16.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include "legged_interface/LeggedInterface.h"
#include "legged_interface/LeggedRobotPreComputation.h"
#include "legged_interface/constraint/FrictionConeConstraint.h"
#include "legged_interface/constraint/LeggedSelfCollisionConstraint.h"
#include "legged_interface/constraint/NormalVelocityConstraintCppAd.h"
#include "legged_interface/constraint/XYReferenceConstraintCppAd.h"
#include "legged_interface/constraint/XYLimitConstraint.h"
#include "legged_interface/constraint/ZeroForceConstraint.h"
#include "legged_interface/constraint/ZeroVelocityConstraintCppAd.h"
#include "legged_interface/cost/LeggedRobotQuadraticTrackingCost.h"
#include "legged_interface/initialization/LeggedRobotInitializer.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/penalties/penalties/DoubleSidedPenalty.h>

#include <legged_interface/dynamics/LeggedRobotDynamicsAD.h>
//感控
#include "legged_interface/constraint/FootCollisionConstraint.h"
#include "legged_interface/PerceptiveLeggedPrecomputation.h"
// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>



namespace legged
{
LeggedInterface::LeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile, bool useHardFrictionConeConstraint)
  : useHardFrictionConeConstraint_(useHardFrictionConeConstraint)
{
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath))
  {
    std::cerr << "[LeggedInterface] Loading task file: " << taskFilePath << std::endl;
  }
  else
  {
    throw std::invalid_argument("[LeggedInterface] Task file not found: " + taskFilePath.string());
  }

  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath))
  {
    std::cerr << "[LeggedInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  }
  else
  {
    throw std::invalid_argument("[LeggedInterface] URDF file not found: " + urdfFilePath.string());
  }

  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath))
  {
    std::cerr << "[LeggedInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  }
  else
  {
    throw std::invalid_argument("[LeggedInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  sqpSettings_ = sqp::loadSettings(taskFile, "sqp", verbose);
  ipmSettings_ = ipm::loadSettings(taskFile, "ipm", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupGridMap(){
  planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();

  double width{5.0}, height{5.0};
  convex_plane_decomposition::PlanarRegion plannerRegion;                                                         // 初始化平面，平面有边界和内嵌区域
  plannerRegion.transformPlaneToWorld.setIdentity();                                                              // 初始化地形坐标和世界坐标重合
  plannerRegion.bbox2d = convex_plane_decomposition::CgalBbox2d(-height / 2, -width / 2, +height / 2, width / 2); // 初始化平面的最大点和最小点(第一象限和第三象限)
  convex_plane_decomposition::CgalPolygonWithHoles2d boundary;                                                    // 平面的端点，初始化4个点(正方形)
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, +width / 2));          // 右上       边界的顶点(CgalPoint2d)，这些顶点构成一个多边形(CgalPolygon2d)
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, +width / 2));          // 左上
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2, -width / 2));          // 左下
  boundary.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2, -width / 2));          // 右下
  plannerRegion.boundaryWithInset.boundary = boundary;
  convex_plane_decomposition::CgalPolygonWithHoles2d insets; // 内嵌偏移区域，可能是用于安全区域
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.02, +width / 2 - 0.02));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.02, +width / 2 - 0.02));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(-height / 2 + 0.02, -width / 2 + 0.02));
  insets.outer_boundary().push_back(convex_plane_decomposition::CgalPoint2d(+height / 2 - 0.02, -width / 2 + 0.02));
  plannerRegion.boundaryWithInset.insets.push_back(insets);
  planarTerrainPtr_->planarRegions.push_back(plannerRegion);

  std::string layer = "elevation_before_postprocess";
  planarTerrainPtr_->gridMap.setGeometry(grid_map::Length(5.0, 5.0), 0.03);                                                // 初始化网格图，大小为5.0，0.03的分辨率(每个小格的大小)
  planarTerrainPtr_->gridMap.add(layer, 0);                                                                                // 原始高程图
  planarTerrainPtr_->gridMap.add("smooth_planar", 0);                                                                      // 平滑层
  signedDistanceFieldPtr_ = std::make_shared<grid_map::SignedDistanceField>(planarTerrainPtr_->gridMap, layer, -0.1, 0.1); // 初始化SDF
}

void LeggedInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                 const std::string& referenceFile, bool verbose)
{
  setupModel(taskFile, urdfFile, referenceFile, verbose); //根据配置文件获取机器人的刚体模型、质心模型
  
  initialState_.setZero(centroidalModelInfo_.stateDim);   //初始状态，用于mpc的warm start，快速计算
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  setupReferenceManager(taskFile, urdfFile, referenceFile, verbose);    //根据配置文件获取机器人步态规划配置

  problemPtr_ = std::make_unique<OptimalControlProblem>();  //优化问题接口

  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(*pinocchioInterfacePtr_, centroidalModelInfo_, "dynamics",
                                                        modelSettings_);
  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);  //优化的动力学模型
  
  // 添加代价函数到优化问题接口
  //跟踪质心模型的参考轨迹，x=[hG,qb,qj]
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose)); 

  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
  {
    const std::string& footName = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr = getEeKinematicsPtr({ footName }, footName);
    
    if (useHardFrictionConeConstraint_)
    {
      problemPtr_->inequalityConstraintPtr->add(footName + "_frictionCone",
                                                getFrictionConeConstraint(i, frictionCoefficient));
    }
    else
    {
      problemPtr_->softConstraintPtr->add(footName + "_frictionCone",
                                          getFrictionConeSoftConstraint(i, frictionCoefficient, barrierPenaltyConfig));   //不等式约束使用软约束，保证有解
    }
    problemPtr_->equalityConstraintPtr->add(
        footName + "_zeroForce",
        std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, i, centroidalModelInfo_)));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getZeroVelocityConstraint(*eeKinematicsPtr, i));    //接触点无滑动约束
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity",
                                            std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(    //足端法线方向速度跟踪约束
                                                *referenceManagerPtr_, *eeKinematicsPtr, i)));
    problemPtr_->softConstraintPtr->add(footName + "_xySwingSoft",
                                        getSoftSwingTrajConstraint(*eeKinematicsPtr, i, taskFile, verbose));      //足端xy方向软约束

    //手不算感控
    if (i < centroidalModelInfo_.numThreeDofContacts - 2){
      // 松弛障碍罚函数初始化
      std::unique_ptr<PenaltyBase> placementPenalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(1e-2, 1e-4)));
      std::unique_ptr<PenaltyBase> collisionPenalty(new RelaxedBarrierPenalty(RelaxedBarrierPenalty::Config(3e-2, 1e-3)));

      // For foot placement   落脚约束
      std::unique_ptr<FootPlacementConstraint> footPlacementConstraint(
          new FootPlacementConstraint(*referenceManagerPtr_, *eeKinematicsPtr, i, numVertices_));
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_footPlacement",
          std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footPlacementConstraint), std::move(placementPenalty))));

      // For foot Collision   足端碰撞约束  摆动时防止足端发生碰撞
      std::unique_ptr<FootCollisionConstraint> footCollisionConstraint(
          new FootCollisionConstraint(*referenceManagerPtr_, *eeKinematicsPtr, signedDistanceFieldPtr_, i, 0.03));
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_footCollision",
          std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footCollisionConstraint), std::move(collisionPenalty))));
    }
      
  }
  
  problemPtr_->softConstraintPtr->add("StateInputLimitSoft", getLimitConstraints(centroidalModelInfo_));    //状态输入极值软约束
  problemPtr_->stateSoftConstraintPtr->add(
      "selfCollision", getSelfCollisionConstraint(*pinocchioInterfacePtr_, taskFile, "selfCollision", verbose));    //自碰撞软约束
  setupPreComputation(taskFile, urdfFile, referenceFile, verbose);
  rolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

  constexpr bool extendNormalizedNomentum = true;
  initializerPtr_ =
      std::make_unique<LeggedRobotInitializer>(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedNomentum);    //MPC求解用
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost>
LeggedInterface::getSoftSwingTrajConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                            size_t contactPointIndex, const std::string& taskFile, bool verbose)
{
  scalar_t weight = 10;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  if (verbose)
  {
    std::cerr << "\n #### SoftSwingTraj Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weight, "softSwingTraj.weight", verbose);
  return std::make_unique<StateInputSoftConstraint>(
      std::make_unique<XYReferenceConstraintCppAd>(*referenceManagerPtr_, eeKinematics, contactPointIndex),
      std::make_unique<QuadraticPenalty>(weight));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupModel(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile, bool /*verbose*/)
{
  // PinocchioInterface
  pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(
      centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
      modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupReferenceManager(const std::string& taskFile, const std::string& urdfFile,
                                            const std::string& referenceFile, bool verbose)
{
  auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(
      loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), planarTerrainPtr_,numVertices_); // 摆动腿设定  地形接口  顶点接口
  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose),
                                                                         std::move(swingTrajectoryPlanner),
                                                                         *pinocchioInterfacePtr_, centroidalModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedInterface::setupPreComputation(const std::string& taskFile, const std::string& urdfFile,
                                          const std::string& referenceFile, bool verbose)
{
  // problemPtr_->preComputationPtr =
  //     std::make_unique<LeggedRobotPreComputation>(*pinocchioInterfacePtr_, centroidalModelInfo_,
  //                                                 *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);
  problemPtr_->preComputationPtr = std::make_unique<PerceptiveLeggedPrecomputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedInterface::loadGaitSchedule(const std::string& file, bool verbose) const
{
  const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

  const auto defaultGait = [defaultModeSequenceTemplate] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1,
                  defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose)
  {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate,
                                        modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info)
{ //输入u=(f1,f2,...,fn,v1,v2,...,vk) n为接触点个数，k为驱动关节个数
  const size_t totalContactDim = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState_, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t base2feetJac(totalContactDim, info.actuatedDofNum);
  for (size_t i = 0; i < info.numThreeDofContacts; i++)
  {
    matrix_t jac = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]),
                                pinocchio::LOCAL_WORLD_ALIGNED, jac); //实质上是相对于世界？
    base2feetJac.block(3 * i, 0, 3, info.actuatedDofNum) = jac.block(0, 6, 3, info.actuatedDofNum);
  }
  
  matrix_t rTaskspace(totalContactDim + totalContactDim, totalContactDim + totalContactDim);
  loadData::loadEigenMatrix(taskFile, "R", rTaskspace);
  matrix_t r(info.inputDim, info.inputDim);
  r.setZero();
  r.topLeftCorner(totalContactDim, totalContactDim) = rTaskspace.topLeftCorner(totalContactDim, totalContactDim);
  // 要优化足端的速度，但是输入是关节的速度，所以需要用雅可比 v_f = Jv, v_f^T*R*v_f = v^T*J^T*R*J*v = v^T*R1*v, R1 = J^T*R*J
  r.block(totalContactDim, totalContactDim, info.actuatedDofNum, info.actuatedDofNum) =
      base2feetJac.transpose() * rTaskspace.block(totalContactDim, totalContactDim, totalContactDim, totalContactDim) * base2feetJac;   
  return r;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedInterface::getBaseTrackingCost(const std::string& taskFile,
                                                                     const CentroidalModelInfo& info, bool verbose)
{
  matrix_t Q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R = initializeInputCostWeight(taskFile, info);

  if (verbose)
  {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::make_unique<LeggedRobotStateInputQuadraticCost>(std::move(Q), std::move(R), info, *referenceManagerPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedInterface::getLimitConstraints(const CentroidalModelInfo& info)
{
  const size_t totalContactDim = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
  const int constraints_num = info.actuatedDofNum + info.actuatedDofNum + info.numThreeDofContacts;   //关节位置、速度，法向力
  vector_t e(constraints_num);
  matrix_t C(constraints_num, info.stateDim);
  matrix_t D(constraints_num, info.inputDim);
  e.setZero();    //偏移量
  C.setZero();    //状态对约束的影响
  D.setZero();    //输入对约束的影响      一个线性系统约束的一般形式Cx+Du+e=0
  C.topRightCorner(info.actuatedDofNum, info.actuatedDofNum).setIdentity();   //关节位置
  D.block(info.actuatedDofNum, totalContactDim, info.actuatedDofNum, info.actuatedDofNum).setIdentity();  //关节速度
  //法向反作用力限幅
  D(info.actuatedDofNum + info.actuatedDofNum + 0, 2) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 1, 5) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 2, 8) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 3, 11) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 4, 14) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 5, 17) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 6, 20) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 7, 23) = 1;
  //手
  D(info.actuatedDofNum + info.actuatedDofNum + 8, 26) = 1;
  D(info.actuatedDofNum + info.actuatedDofNum + 9, 29) = 1;

  std::vector<std::unique_ptr<PenaltyBase>> state_input_limit_penalty;

  state_input_limit_penalty.resize(constraints_num);    //定义状态输入限幅惩罚
  RelaxedBarrierPenalty::Config pos_limit_barrier_penalty_config(1, 0.1); //状态惩罚
  RelaxedBarrierPenalty::Config vel_limit_barrier_penalty_config(1, 0.1); //状态惩罚
  RelaxedBarrierPenalty::Config force_limit_barrier_penalty_config(0.1, 1); //输入惩罚
  const auto& model = pinocchioInterfacePtr_->getModel(); //获取模型参数，得到模型的限幅值，比如关节限位、电机的最大转速....
  //双边惩罚，约束解在限定的范围内
  for (int j = 0; j < info.actuatedDofNum; j++)
  {
    state_input_limit_penalty[j] =
        std::make_unique<DoubleSidedPenalty>(model.lowerPositionLimit(6 + j), model.upperPositionLimit(6 + j),
                                             std::make_unique<RelaxedBarrierPenalty>(pos_limit_barrier_penalty_config));
    state_input_limit_penalty[info.actuatedDofNum + j] =
        std::make_unique<DoubleSidedPenalty>(-model.velocityLimit(6 + j), model.velocityLimit(6 + j),
                                             std::make_unique<RelaxedBarrierPenalty>(vel_limit_barrier_penalty_config));
  }
  for (int leg = 0; leg < info.numThreeDofContacts; leg++)
  {
    //接触力约束在0和350之间
    state_input_limit_penalty[info.actuatedDofNum + info.actuatedDofNum + leg] = std::make_unique<DoubleSidedPenalty>(
        0, 350, std::make_unique<RelaxedBarrierPenalty>(force_limit_barrier_penalty_config));
  }

  return std::make_unique<StateInputSoftConstraint>(std::make_unique<LinearStateInputConstraint>(e, C, D),
                                                    std::move(state_input_limit_penalty));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config>
LeggedInterface::loadFrictionConeSettings(const std::string& taskFile, bool verbose)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t frictionCoefficient = 1;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose)
  {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose)
  {
    std::cerr << " #### =============================================================================\n";
  }

  return { frictionCoefficient, barrierPenaltyConfig };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedInterface::getFrictionConeConstraint(size_t contactPointIndex,
                                                                                 scalar_t frictionCoefficient)
{
  FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  return std::make_unique<FrictionConeConstraint>(*referenceManagerPtr_, frictionConeConConfig, contactPointIndex,
                                                  centroidalModelInfo_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedInterface::getFrictionConeSoftConstraint(
    size_t contactPointIndex, scalar_t frictionCoefficient, const RelaxedBarrierPenalty::Config& barrierPenaltyConfig)
{
  return std::make_unique<StateInputSoftConstraint>(getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
                                                    std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfig));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<EndEffectorKinematics<scalar_t>>
LeggedInterface::getEeKinematicsPtr(const std::vector<std::string>& footNames, const std::string& modelName)
{
  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

  const auto infoCppAd = centroidalModelInfo_.toCppAd();
  const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };
  eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(
      *pinocchioInterfacePtr_, pinocchioMappingCppAd, footNames, centroidalModelInfo_.stateDim,
      centroidalModelInfo_.inputDim, velocityUpdateCallback, modelName, modelSettings_.modelFolderCppAd,
      modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

  return eeKinematicsPtr;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedInterface::getZeroVelocityConstraint(
    const EndEffectorKinematics<scalar_t>& eeKinematics, size_t contactPointIndex)
{
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    config.b(2) += -3 * 0.02;
    config.Ax.setZero(3, 3);
    config.Ax(2, 2) = 3;
    return config;
  };
  return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(
      *referenceManagerPtr_, eeKinematics, contactPointIndex, eeZeroVelConConfig(modelSettings_.positionErrorGain)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> LeggedInterface::getSelfCollisionConstraint(const PinocchioInterface& pinocchioInterface,
                                                                       const std::string& taskFile,
                                                                       const std::string& prefix, bool verbose)
{
  std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  scalar_t mu = 1e-2;
  scalar_t delta = 1e-3;
  scalar_t minimumDistance = 0.0;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  if (verbose)
  {
    std::cerr << "\n #### SelfCollision Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, mu, prefix + ".mu", verbose);
  loadData::loadPtreeValue(pt, delta, prefix + ".delta", verbose);
  loadData::loadPtreeValue(pt, minimumDistance, prefix + ".minimumDistance", verbose);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionObjectPairs", collisionObjectPairs, verbose);
  loadData::loadStdVectorOfPair(taskFile, prefix + ".collisionLinkPairs", collisionLinkPairs, verbose);

  geometryInterfacePtr_ =
      std::make_unique<PinocchioGeometryInterface>(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
  if (verbose)
  {
    std::cerr << " #### =============================================================================\n";
    const size_t numCollisionPairs = geometryInterfacePtr_->getNumCollisionPairs();
    std::cerr << "SelfCollision: Testing for " << numCollisionPairs << " collision pairs\n";
  }

  std::unique_ptr<StateConstraint> constraint = std::make_unique<LeggedSelfCollisionConstraint>(
      CentroidalModelPinocchioMapping(centroidalModelInfo_), *geometryInterfacePtr_, minimumDistance);

  auto penalty = std::make_unique<RelaxedBarrierPenalty>(RelaxedBarrierPenalty::Config{ mu, delta });

  return std::make_unique<StateSoftConstraint>(std::move(constraint), std::move(penalty));
}

}  // namespace legged
