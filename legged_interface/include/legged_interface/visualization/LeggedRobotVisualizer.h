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

#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <legged_interface/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

#include "legged_interface/foot_planner/SwingTrajectoryPlanner.h"

namespace legged
{
using namespace ocs2;
using namespace ocs2::legged_robot;

class LeggedRobotVisualizer : public DummyObserver
{
public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "odom";              // Frame name all messages are published in
  scalar_t footMarkerDiameter_ = 0.03;        // Size of the spheres at the feet
  scalar_t footAlphaWhenLifted_ = 0.3;        // Alpha value when a foot is lifted.
  scalar_t forceScale_ = 150.0;               // Vector scale in N/m
  scalar_t velScale_ = 5.0;                   // Vector scale in m/s
  scalar_t copMarkerDiameter_ = 0.03;         // Size of the sphere at the center of pressure
  scalar_t supportPolygonLineWidth_ = 0.005;  // LineThickness for the support polygon
  scalar_t trajectoryLineWidth_ = 0.01;       // LineThickness for trajectories
  std::vector<Color> feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple, 
                                      Color::green, Color::red, Color::yellow, Color::blue}; // Colors for markers per feet

  /**
   *
   * @param pinocchioInterface
   * @param n
   * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
   */
  LeggedRobotVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                        const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                        scalar_t maxUpdateFrequency = 100.0);

  ~LeggedRobotVisualizer() override = default;
  void update(const SystemObservation& observation);

  void update(const SystemObservation& observation, const PrimalSolution& primalSolution,
              const CommandData& command) override;

  void update(const SystemObservation &observation, const PrimalSolution &primalSolution, const CommandData &command,
              const std::shared_ptr<SwingTrajectoryPlanner> &swingTrajectoryPlanner,
              const vector3_t &linearMomentumDes,
              const vector_t &rbdState /*const std::vector<vector_t> &rbdStateTraj*/,
              const vector3_t dcmDesired, const vector3_t dcmMeasured,
              const vector6_t footDes, const vector6_t footMea);

  void update(const SystemObservation& observation, const CommandData& command);

  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed = 1.0);

  void publishObservation(const ros::Time& timeStamp, const SystemObservation& observation);

  void publishReferenceTrajectory(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories);

  void publishDesiredTrajectory(const ros::Time& timeStamp, const TargetTrajectories& targetTrajectories,
                                const std::shared_ptr<SwingTrajectoryPlanner>& swingTrajectoryPlanner);

  void publishOptimizedStateTrajectory(const ros::Time& timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                       const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule);

  void publishDCMPosition(const ros::Time &timeStamp, const vector_t &rbdState /*const std::vector<vector_t> &rbdStateTraj*/,const vector3_t dcmDesired, const vector3_t dcmMeasured);
  void publishfootPosition(const ros::Time &timeStamp,const vector6_t footDesired, const vector6_t footMeasured);
  void publishDesiredLinearMomentum(const vector3_t &linearMomentumDes);

private:
  LeggedRobotVisualizer(const LeggedRobotVisualizer&) = delete;
  void publishJointTransforms(const ros::Time& timeStamp, const vector_t& jointAngles,
                              const std::string& tf_prefix = "") const;
  void publishBaseTransform(const ros::Time& timeStamp, const vector_t& basePose, const std::string& tf_prefix = "");
  void publishCartesianMarkers(const ros::Time& timeStamp, const contact_flag_t& contactFlags,
                               const std::vector<vector3_t>& feetPositions,
                               const std::vector<vector3_t>& feetForces) const;

  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

  tf::TransformBroadcaster tfBroadcaster_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

  ros::Publisher costDesiredBasePositionPublisher_;
  std::vector<ros::Publisher> costDesiredFeetPositionPublishers_;

  ros::Publisher desiredTrajPublishers_;

  ros::Publisher stateOptimizedPublisher_;

  ros::Publisher currentStatePublisher_;

  ros::Publisher swingTrajPlotPublisher_;

  ros::Publisher linearMomentumDesirePlotPublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
  bool visualize_ik_flag_{};
};

}  // namespace legged