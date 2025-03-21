/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#include "legged_interface/gait/GaitSchedule.h"

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitSchedule::GaitSchedule(ModeSchedule initModeSchedule, ModeSequenceTemplate initModeSequenceTemplate,
                           scalar_t phaseTransitionStanceTime)    //来自reference.info
  : modeSchedule_(std::move(initModeSchedule))      //initModeSchedule = initialModeSchedule
  , modeSequenceTemplate_(std::move(initModeSequenceTemplate))    //initModeSequenceTemplate = defaultModeSequenceTemplate
  , phaseTransitionStanceTime_(phaseTransitionStanceTime) // phaseTransitionStanceTime = 0.4 （来自modelsetting）
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::insertModeSequenceTemplate(const ModeSequenceTemplate& modeSequenceTemplate, scalar_t startTime,
                                              scalar_t finalTime)//把步态模板(gait.info)插入到步态
{
  modeSequenceTemplate_ = modeSequenceTemplate;     //插入的步态模板 trot
  auto& eventTimes = modeSchedule_.eventTimes;      //原来的步态
  auto& modeSequence = modeSchedule_.modeSequence;

  // find the index on which the new gait should be added
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), startTime) - eventTimes.begin();

  // delete the old logic from the index
  if (index < eventTimes.size())
  {
    eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
    modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
  }

  // add an intermediate stance phase
  scalar_t phaseTransitionStanceTime = phaseTransitionStanceTime_;    //0.4
  if (!modeSequence.empty() && modeSequence.back() == ModeNumber::STANCE)
  {
    phaseTransitionStanceTime = 0.0;
  }

  if (phaseTransitionStanceTime > 0.0)
  {
    eventTimes.push_back(startTime);
    modeSequence.push_back(ModeNumber::STANCE);
  }

  // tile the mode sequence template from startTime+phaseTransitionStanceTime to finalTime.
  tileModeSequenceTemplate(startTime + phaseTransitionStanceTime, finalTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule GaitSchedule::getModeSchedule(scalar_t lowerBoundTime, scalar_t upperBoundTime)    //规划从下界时间到上界时间的步态
{
  auto& eventTimes = modeSchedule_.eventTimes;    //初始eventTimes = {[0]0.5}
  auto &modeSequence = modeSchedule_.modeSequence; // 初始modeSequence = {[0]STANCE,[1]STANCE}
  const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(), lowerBoundTime) - eventTimes.begin();
  // std::cout << "*****************index*****************" << std::endl;
  // std::cout << index << std::endl;
  if (index > 0)
  {
    // delete the old logic from index and set the default start phase to stance
    eventTimes.erase(eventTimes.begin(),
                     eventTimes.begin() + index - 1);  // keep the one before the last to make it stance
    modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);

    // set the default initial phase
    modeSequence.front() = ModeNumber::STANCE;    //使第一个相位的步态为STANCE
  }

  // Start tiling at time
  const auto tilingStartTime = eventTimes.empty() ? lowerBoundTime : eventTimes.back();

  // delete the last default stance phase
  eventTimes.erase(eventTimes.end() - 1, eventTimes.end());   //删除最后一个元素
  modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

  // tile the template logic
  tileModeSequenceTemplate(tilingStartTime, upperBoundTime);    //扩张步态到上界时间
  return modeSchedule_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitSchedule::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime)
{
  auto& eventTimes = modeSchedule_.eventTimes;      //原步态
  auto& modeSequence = modeSchedule_.modeSequence;
  const auto &templateTimes = modeSequenceTemplate_.switchingTimes;       //初始步态switchingTimes = {0.0，0.1}   根据插入的模板来(gait.info)
  const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;  //初始步态modeSequence = {STANCE}  
  const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size(); //初始步态numTemplateSubsystems = 1

  // If no template subsystem is defined, the last subsystem should continue for ever
  if (numTemplateSubsystems == 0)
  {
    return;
  }

  if (!eventTimes.empty() && startTime <= eventTimes.back())
  {
    throw std::runtime_error("The initial time for template-tiling is not greater than the last event time.");
  }

  // add a initial time
  eventTimes.push_back(startTime);

  // concatenate from index
  while (eventTimes.back() < finalTime)   //直到eventTimes更新到终端时间为之，否则会循环插入步态
  {
    for (size_t i = 0; i < templateModeSequence.size(); i++)
    {
      modeSequence.push_back(templateModeSequence[i]);      //modeSequence更新
      scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
      eventTimes.push_back(eventTimes.back() + deltaTime);      //eventTimes更新
    }  // end of i loop
  }    // end of while loop

  // default final phase
  modeSequence.push_back(ModeNumber::STANCE);
}

}  // namespace legged_robot
}  // namespace ocs2
