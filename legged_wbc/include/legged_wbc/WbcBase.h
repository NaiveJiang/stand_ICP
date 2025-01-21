//
// Created by qiayuan on 2022/7/1.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <legged_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
namespace legged
{
  using namespace ocs2;
  using namespace legged_robot;

  // Ax -b = w
  // Dx - f <= v
  // w -> 0, v -> 0

  // Decision Variables: x = [\dot u^T, 3*F(3)^T, \tau^T]^T , \dot u in local frame
  #define USE_6_AXIS_FOOT 1
  #define USE_6_AXIS_HAND 0
  class WbcBase
  {
    using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
    using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;

  public:
    WbcBase(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
            const PinocchioEndEffectorKinematics &eeKinematics, const PinocchioInterface &pinocchioInterfaceQuat);

    virtual void loadTasksSetting(const std::string &taskFile, bool verbose);

    virtual vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                            size_t mode, scalar_t period, const SystemObservation &observation);
    void setCmdBodyPosVel(const vector_t &cmd_body_pos, const vector_t &cmd_body_vel)
    {
      cmd_body_pos_ = cmd_body_pos;
      cmd_body_vel_ = cmd_body_vel;
    }

    void setEarlyLateContact(const std::array<contact_flag_t, 2> &early_late_contact)
    {
      earlyLatecontact_ = early_late_contact;
    }

    void setFootPosVelAccDesired(const std::array<vector_t, 3> &footPosVelAccDesired)
    {
      footPosVelAccDesired_ = footPosVelAccDesired;
    }
    void setJointAccDesired(const vector_t &jointAccDesired)
    {
      jointAccDesired_ = jointAccDesired;
    }
    void setKpKd(scalar_t swingKp, scalar_t swingKd)
    {
      swingKp_ = swingKp;
      swingKd_ = swingKd;
    }
    size_t getContactForceSize()
    {
      return contact_force_size_;
    }
    void setStanceMode(bool stance_mode)
    {
      stance_mode_ = stance_mode;
    }

    vector3_t getLinearMomentumDesired()
    {
      return linearMomentumDesired;
    }
    scalar_t contactMotionScale_;
    bool init_flg;
    Eigen::Matrix<scalar_t, 3, 1> ric_des;
    Eigen::Matrix<scalar_t, 3, 1> i_ric;
    vector3_t copPostion;

    std::vector<vector3_t> feetForcesMeasure;
    std::vector<vector3_t> feetTorqueMeasure;
    matrix3_t skewSymmetric(const vector3_t &v)
    {
      matrix3_t antisymmetricMatrix;
      antisymmetricMatrix << 0, -v(2), v(1),
          v(2), 0, -v(0),
          -v(1), v(0), 0;
      return antisymmetricMatrix;
    }

    vector6_t getGroundWrench(const SystemObservation &observation);

    void set_dcm_Height(scalar_t set)
    {
      dcmDesired_.z() = set;
    }

    void set_dcm_x(scalar_t set)
    {
      dcmDesired_.x() = dcmDesired_.x() + set;
    }

    vector3_t get_dcm_des()
    {
      return dcmDesired_;
    }

    vector3_t get_dcm_mea()
    {
      return dcmMeasured_;
    }

    vector6_t get_foot_des()
    {
      return footPosDes_;
    }

    vector6_t get_foot_mea()
    {
      return footPosMea_;
    }

  protected:
    void updateMeasured(const vector_t &rbdStateMeasured, const SystemObservation &observation);
    void updateDesired(const vector_t &stateDesired, const vector_t &inputDesired);

    size_t getNumDecisionVars() const
    {
      return numDecisionVars_;
    }

    Task formulateFloatingBaseEomTask();
    Task formulateTorqueLimitsTask();
    Task formulateNoContactMotionTask();
    Task formulateFrictionConeTask();
    Task formulateBaseHeightMotionTask();
    Task formulateBaseAngularMotionTask();
    Task formulateBaseXYLinearAccelTask();
    Task formulateBaseXYLinearAccelTaskInStance(scalar_t period);
    Task formulateBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired, scalar_t period);
    Task formulateSwingLegTask();
    Task formulateContactForceTask(const vector_t &inputDesired) const;
    Task formulateZeroTorque();

    Task formulateCentroidalDynamicsTask();
    Task formulateMomentumTask(scalar_t period);
    Task formulateBaseHightAccTask();

    Task formulateJointAccSmallTask();
    Task formulateGRFSmallTask();

    Task formulateArmTask();
    vector_t armPosDesired_, armVelDesired_;
    scalar_t armKp_{}, armKd_{};

    Task formulatefootTask();
    vector6_t footPosDes_, footPosMea_;
    matrix_t jf, djf;
    scalar_t footLKp_{}, footLKd_{};
#if USE_6_AXIS_FOOT
    vector_t footVelDes_, footVelMea_;
    matrix_t footRotDes_, footRotMea_;
    scalar_t footAKp_{}, footAKd_{};
#else
    vector6_t footVelDes_, footVelMea_;
#endif

#if USE_6_AXIS_HAND
    Task formulateHandTask();
    vector6_t handPosDes_, handPosMea_;
    matrix_t jh, djh;
    scalar_t handLKp_{}, handLKd_{};
    scalar_t handAKp_{}, handAKd_{};
    vector_t handVelDes_, handVelMea_;
    matrix_t handRotDes_, handRotMea_;
#endif


    void compensateFriction(vector_t &x);

    size_t numDecisionVars_;
    PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
    // test
    PinocchioInterface pinocchioInterfaceMeasuredQuat_;
    CentroidalModelInfo info_;

    std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
    CentroidalModelPinocchioMapping mapping_;
    CentroidalModelRbdConversions rbdConversions_;

    vector_t qMeasured_, vMeasured_, inputLast_;
    // test
    vector_t qMeasuredQ_;

    matrix_t j_, dj_;
    Matrix6x base_j_, base_dj_;
    contact_flag_t contactFlag_{};
    size_t numContacts_{};

    vector_t torqueLimits_;
    scalar_t frictionCoeff_{}, swingKp_{}, swingKd_{};
    scalar_t baseHeightKp_{}, baseHeightKd_{};
    scalar_t baseAngularKp_{}, baseAngularKd_{};

    // test
    scalar_t basexyKp_{}, basexyKd_{};
    scalar_t momLKp_{}, momLKd_{}, momLKi_{};
    scalar_t momLKzp_{}, momLKzd_{};
    scalar_t momAKp_{}, momAKd_{};
    vector3_t linearMomentumDesired;

    vector3_t dcmDesired_, dcmVelDesired_;
    vector3_t dcmMeasured_, dcmVelMeasured_;
    vector3_t comPosition_, comVelocity_;
    matrix_t Ag, dAg;
    vector6_t hg, dhg;
    scalar_t omega0{};
    scalar_t mass_{};

    vector_t cmd_body_pos_;
    vector_t cmd_body_vel_;
    scalar_t com_kp_{}, com_kd_{};
    Vector6 basePoseDes_, baseVelocityDes_, baseAccelerationDes_;

    std::array<contact_flag_t, 2> earlyLatecontact_;

    std::vector<vector3_t> footPosDesired_, footVelDesired_;
    std::array<vector_t, 3> footPosVelAccDesired_;

    vector_t jointAccDesired_;
    size_t contact_force_size_ = 0;
    bool stance_mode_ = false;
  };

} // namespace legged