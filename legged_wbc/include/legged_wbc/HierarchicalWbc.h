//
// Created by qiayuan on 22-12-23.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include "legged_wbc/WbcBase.h"

namespace legged
{
class HierarchicalWbc : public WbcBase
{
public:
  using WbcBase::WbcBase;

  vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                  size_t mode, scalar_t period, const SystemObservation &observation) override;

  void loadTasksSetting(const std::string &taskFile, bool verbose) override;

  Task formulateStanceBaseAccelTask(const vector_t &stateDesired, const vector_t &inputDesired,
                                    scalar_t period);

  Task formulateZeroTorque();

  scalar_t swingLeg_;
  scalar_t baseAccel_;
  scalar_t contactForce_;
  scalar_t zeroTorque_;
};

}  // namespace legged
