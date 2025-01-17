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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "legged_interface/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/Numerics.h>

#include <legged_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <geometry_msgs/Twist.h>

//感控
#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>
#include <iostream>
#include <iomanip>

namespace ocs2
{
namespace legged_robot
{
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config,std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr, size_t numVertices)
  : config_(std::move(config))
  , feet_bias_{}
  , leg_swing_down_flags_{}
  , feetXTrajsBuf_{ std::move(feet_array_t<std::vector<MultiCubicSpline>>{}) }
  , feetYTrajsBuf_{ std::move(feet_array_t<std::vector<MultiCubicSpline>>{}) }
  , feetZTrajsBuf_{ std::move(feet_array_t<std::vector<MultiCubicSpline>>{}) }
  , footTrajsEventsBuf_{ std::move(std::vector<scalar_t>{}) }
  , StartStopTimeBuf_{ std::move(feet_array_t<std::vector<std::array<scalar_t, 2>>>{}) }
  , body_vel_world_buf_{ std::move(vector_t(6)) }
  , swing_cmd_buf_{ std::move(vector3_t(0, 0, 0)) }
  , current_feet_position_buf_{ std::move(feet_array_t<vector3_t>{}) }
  , latestStanceposition_{}
  , numVertices_(numVertices)
  , planarTerrainPtr_(std::move(planarTerrainPtr))
{
  numFeet_ = feet_bias_.size();   //足端接触点的数量 8个    feet_bias为初始时足端点相对于机体的偏移    脚尖相距脚跟0.9

  feet_bias_[0] << config.feet_bias_x1, config.feet_bias_y, config.feet_bias_z;   // L1   脚尖
  feet_bias_[1] << config.feet_bias_x1, -config.feet_bias_y, config.feet_bias_z;  // R1

  feet_bias_[2] << config.feet_bias_x2, config.feet_bias_y, config.feet_bias_z; // L2     脚跟
  feet_bias_[3] << config.feet_bias_x2, -config.feet_bias_y, config.feet_bias_z; // R2

  feet_bias_[4] << config.feet_bias_x1, config.feet_bias_y, config.feet_bias_z; // L3   脚尖
  feet_bias_[5] << config.feet_bias_x1, -config.feet_bias_y, config.feet_bias_z; // R3

  feet_bias_[6] << config.feet_bias_x2, config.feet_bias_y, config.feet_bias_z;   //L4    脚跟
  feet_bias_[7] << config.feet_bias_x2, -config.feet_bias_y, config.feet_bias_z;  //R4

  body_vel_cmd_.resize(6);
  body_vel_cmd_.setZero();

  std::cout.setf(std::ios::fixed);
  std::cout.precision(6);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXvelocityConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return feetXTrajs_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYvelocityConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return feetYTrajs_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return feetZTrajs_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXpositionConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  auto val = feetXTrajs_[leg][index].position(time);
  return val;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYpositionConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  auto val = feetYTrajs_[leg][index].position(time);
  return val;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);   //查找当前时间对应的相位
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  auto val = feetZTrajs_[leg][index].position(time);    //在对应相位里寻找当前时间所处的位置
  return val;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::array<scalar_t, 2> SwingTrajectoryPlanner::getSwingStartStopTime(size_t leg, scalar_t time) const
{
  auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  index = std::min((int)(feetTrajsEvents_[leg].size() - 1), index);
  return startStopTime_[leg][index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule, const TargetTrajectories &targetTrajectories,
                                    scalar_t initTime, const vector_t &initState)
{
  //感控
  planarTerrain_ = *planarTerrainPtr_;  //拷贝地形信息
  
  const auto& modeSequence = modeSchedule.modeSequence;   //步态列表
  const auto& eventTimes = modeSchedule.eventTimes;   //步态切换时间表
  body_vel_world_buf_.updateFromBuffer();
  const auto& body_vel_world = body_vel_world_buf_.get();   //当前机体速度
  current_feet_position_buf_.updateFromBuffer();    
  const auto& current_feet_position = current_feet_position_buf_.get(); //当前足端位置
  contact_flag_t cmd_state_leg = modeNumber2StanceLeg(modeSchedule.modeAtTime(initTime + 0.001));   //当前时间下属于的步态相位对应足端支撑点状态

  for (int i = 0; i < numFeet_; i++)
  {
    latestStanceposition_[i] = cmd_state_leg[i] ? current_feet_position[i] : latestStanceposition_[i];    //只有接触地面才会更新Stanceposition(当前站立位置) 
    latestStanceposition_[i].z() = config_.next_position_z;
  }

  feet_array_t<vector3_t> last_stance_position = latestStanceposition_;   //支撑点位置更新
  feet_array_t<vector3_t> next_stance_position = latestStanceposition_;
  
  feet_array_t<int> last_final_idx{};
  feet_array_t<int> last_stand_final_idx{};

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);    //得到步态列表所有步态对应的接触点情况
  eesContactFlagStocks_ = eesContactFlagStocks; //给全局空间

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++)
  {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);   //得到该接触点状态在步态列表的开始相位和结束相位
  }

  // std::cout << "**************size**************" << std::endl;
  // std::cout << modeSequence.size() << std::endl;

  //*************************************感控*************************************
  feet_array_t<vector3_t> next_stance_projection = latestStanceposition_;
  feet_array_t<vector3_t> latest_stance_projection = latestStanceposition_;
  feet_array_t<int> last_proj_final_idx{};

   for (size_t j = 0; j < numFeet_; j++){
      nextStandProjections_[j].clear();
      next_middle_body_pos_seq_[j].clear();
      convexPolygonsInNextStand_[j].clear(); // 在支撑时的凸平面

      nextStandProjections_[j].reserve(modeSequence.size());
      next_middle_body_pos_seq_[j].reserve(modeSequence.size());
      convexPolygonsInNextStand_[j].reserve(modeSequence.size());
   }


  if (start_elevation_map_){
    for (size_t j = 0; j < numFeet_; j++){
      // nextStandProjections_[j].clear();
      // next_middle_body_pos_seq_[j].clear();
      // // convexPolygonsInNextStand_[j].clear(); // 在支撑时的凸平面

      // nextStandProjections_[j].reserve(modeSequence.size());
      // next_middle_body_pos_seq_[j].reserve(modeSequence.size());
      // // convexPolygonsInNextStand_[j].reserve(modeSequence.size());

      for (int p = 0; p < modeSequence.size(); ++p){
        if (!eesContactFlagStocks[j][p]){
          const int swingStartIndex = startTimesIndices[j][p]; // 摆动开始的相位(支撑结束的相位)
          const int swingFinalIndex = finalTimesIndices[j][p]; // 摆动结束的相位(下一次支撑的开始)
          checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

          const scalar_t swingStartTime = eventTimes[swingStartIndex]; // 相位对应的时间节点
          const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

          // 只会规划当前时间下所处的步态，当前时间之前的步态不考虑
          if (initTime < swingFinalTime && swingFinalIndex > last_proj_final_idx[j]){// 当前时间小于摆动结束时间(正在摆动中)，当前摆动结束索引大于上一次结束索引（新的一次摆动动作，还未计算过落足点，计算过一次不用再算了）

            latest_stance_projection[j] = next_stance_projection[j]; // 摆动前落足点位置

            scalar_t next_middle_time = 0;
            scalar_t nextStanceFinalTime = 0;
            if (swingFinalIndex < modeSequence.size() - 1){ // 步态是切换的，且步态在列表内
              const int nextStanceFinalIndex = finalTimesIndices[j][swingFinalIndex + 1]; // 下一次支撑点的结束相位为当前摆动相位结束的下一步
              nextStanceFinalTime = eventTimes[nextStanceFinalIndex];                     // 下一次支撑结束时间点（也是下一次摆动开始的时间点）
              next_middle_time = (swingFinalTime + nextStanceFinalTime) / 2;              // 中间时间 摆动结束的时刻(下一次支撑点开始时刻)到下一次支撑点结束时刻的中间时刻(为了计算落足点)
            }
            else{
              next_middle_time = swingFinalTime;
            }

            vector_t next_middle_body_pos = targetTrajectories.getDesiredState(next_middle_time).segment<6>(6); // 参考轨迹中间时间点机体位置
            // 感控
            next_middle_body_pos_seq_[j].push_back(next_middle_body_pos);

            vector_t current_body_pos = targetTrajectories.getDesiredState(initTime).segment<6>(6); // 参考轨迹当前时间下机体位置
            vector_t current_body_vel = targetTrajectories.stateTrajectory[0].segment<3>(0);        // 参考轨迹起点速度(给定速度)

            next_stance_projection[j] = calNextFootPos(j, initTime, swingFinalTime, next_middle_time, next_middle_body_pos,
                                                       current_body_pos, current_body_vel); // 计算落足点

            auto penaltyFunction = [](const vector3_t & /*projectedPoint*/){ return 0.0; }; // 投影代价用的罚函数
            // 找到标准落脚点在所有分割平面的投影点，选取最优的投影点(x,y)，并将给投影点表示为世界坐标系(x,y,z)，作为真正的的落脚点
            auto projection = getBestPlanarRegionAtPositionInWorld(next_stance_projection[j], planarTerrain_.planarRegions, penaltyFunction);

            // if (j == 2 || j == 3){ // 脚跟
            //   // 比较平面是否与脚尖相同
            //   if (j == 2){
            //     if (projection.regionPtr != nullptr && nextStandProjections_[0][p].regionPtr != nullptr){
            //       if (projection.regionPtr != nextStandProjections_[0][p].regionPtr){   // 出现平面不同
            //         // convex_plane_decomposition::PlanarRegion regionCopy = *(nextStandProjections_[0][p].regionPtr);
            //         // projection = calPositionInRegion(next_stance_projection[j], regionCopy); // 计算新的落足点
            //         const Eigen::Vector3d positionInTerrainFrame = nextStandProjections_[0][p].regionPtr->transformPlaneToWorld.inverse() * next_stance_projection[j]; // 得到标准落足点在指定平面坐标系的坐标
            //         const auto projectedPointInTerrainFrame = projectToPlanarRegion({positionInTerrainFrame.x(), positionInTerrainFrame.y()}, *(nextStandProjections_[0][p].regionPtr)); // 计算指定平面的投影
            //         const auto projectionInWorldFrame =
            //             convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(projectedPointInTerrainFrame, nextStandProjections_[0][p].regionPtr->transformPlaneToWorld); // 把投影点转到世界坐标系
            //         projection.regionPtr = nextStandProjections_[0][p].regionPtr;
            //         projection.positionInTerrainFrame = projectedPointInTerrainFrame;
            //         projection.positionInWorld = projectionInWorldFrame;
            //         // 要把脚尖的投影点前移
            //         vector3_t bias = {0.09, 0, 0};
            //         nextStandProjections_[0][p].positionInWorld = calBiasPos2d(bias, nextStandProjections_[0][p].positionInWorld, next_middle_body_pos_seq_[0][p]);
            //         // 再计算脚尖投影在平面坐标的表示
            //         auto next_proj_in_terrainFrame = nextStandProjections_[0][p].regionPtr->transformPlaneToWorld.inverse() * nextStandProjections_[0][p].positionInWorld;
            //         nextStandProjections_[0][p].positionInTerrainFrame = {next_proj_in_terrainFrame.x(), next_proj_in_terrainFrame.y()};
            //       }
            //     }
            //   }
            //   else{
            //     if (projection.regionPtr != nullptr && nextStandProjections_[1][p].regionPtr != nullptr){
            //       if (projection.regionPtr != nextStandProjections_[1][p].regionPtr){ // 出现平面不同
            //         const Eigen::Vector3d positionInTerrainFrame = nextStandProjections_[1][p].regionPtr->transformPlaneToWorld.inverse() * next_stance_projection[j];                   // 得到标准落足点在指定平面坐标系的坐标
            //         const auto projectedPointInTerrainFrame = projectToPlanarRegion({positionInTerrainFrame.x(), positionInTerrainFrame.y()}, *(nextStandProjections_[1][p].regionPtr)); // 计算指定平面的投影
            //         const auto projectionInWorldFrame =
            //             convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(projectedPointInTerrainFrame, nextStandProjections_[1][p].regionPtr->transformPlaneToWorld); // 把投影点转到世界坐标系
            //         projection.regionPtr = nextStandProjections_[1][p].regionPtr;
            //         projection.positionInTerrainFrame = projectedPointInTerrainFrame;
            //         projection.positionInWorld = projectionInWorldFrame;
            //         // 要把脚尖的投影点前移
            //         vector3_t bias = {0.09, 0, 0};
            //         nextStandProjections_[1][p].positionInWorld = calBiasPos2d(bias, nextStandProjections_[1][p].positionInWorld, next_middle_body_pos_seq_[1][p]);
            //         // 再计算脚尖投影在平面坐标的表示
            //         auto next_proj_in_terrainFrame = nextStandProjections_[1][p].regionPtr->transformPlaneToWorld.inverse() * nextStandProjections_[1][p].positionInWorld;
            //         nextStandProjections_[1][p].positionInTerrainFrame = {next_proj_in_terrainFrame.x(), next_proj_in_terrainFrame.y()};
            //       }
            //     }
            //   }
            // }
            if(j==2||j==3){
              scalar_t growthFactor = 1.05;
              // 从落脚点为中心，在落脚点的平面上扩展出一个凸平面
              const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                  projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numVertices_, growthFactor);

              convexPolygonsInNextStand_[j][p] = convexRegion;
            }


            nextStandProjections_[j][p] = projection; // 更新投影点
            last_proj_final_idx[j] = swingFinalIndex;
          }
        }
      }
    }
  }

    //*************************************感控*************************************
    for (size_t j = 0; j < numFeet_; j++) // j = 足端点索引号
    {
      feetXTrajs_[j].clear(); // 轨迹
      feetYTrajs_[j].clear();
      feetZTrajs_[j].clear();
      startStopTime_[j].clear();
      feetXTrajs_[j].reserve(modeSequence.size());
      feetYTrajs_[j].reserve(modeSequence.size());
      feetZTrajs_[j].reserve(modeSequence.size());
      startStopTime_[j].reserve(modeSequence.size());

      // 感控
      StandProjections_[j].clear();
      convexPolygonsInCurrentStand_[j].clear();
      feet_front_[j].clear();
      middleTimes_[j].clear();

      StandProjections_[j].reserve(modeSequence.size());
      convexPolygonsInCurrentStand_[j].reserve(modeSequence.size());
      feet_front_[j].reserve(modeSequence.size());

      initStandFinalTime_[j] = 0;

      vector3_t last_stance_point;
      last_stance_point(0) = 0;
      last_stance_point(1) = 0;
      last_stance_point(2) = 0;
      for (int p = 0; p < modeSequence.size(); ++p) // p = 步态列表的步态索引号
      {
        if (!eesContactFlagStocks[j][p])                       // 当前步态下该足端点未接触到地面(摆动腿)
        {                                                      // for a swing leg
          const int swingStartIndex = startTimesIndices[j][p]; // 摆动开始的相位(支撑结束的相位)
          const int swingFinalIndex = finalTimesIndices[j][p]; // 摆动结束的相位(下一次支撑的开始)
          checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

          const scalar_t swingStartTime = eventTimes[swingStartIndex]; // 相位对应的时间节点
          const scalar_t swingFinalTime = eventTimes[swingFinalIndex];
          const scalar_t time_length = swingFinalTime - swingStartTime;  // 一次动作的间隔
          startStopTime_[j].push_back({swingStartTime, swingFinalTime}); // 摆动时开始和结束时间

          // 只会规划当前时间下所处的步态，当前时间之前的步态不考虑
          if (initTime < swingFinalTime && swingFinalIndex > last_final_idx[j]) // 当前时间小于摆动结束时间(正在摆动中)，当前摆动结束索引大于上一次结束索引（新的一次摆动动作，还未计算过落足点，计算过一次不用再算了）
          {
            last_stance_position[j] = next_stance_position[j]; // 摆动前落足点位置

            scalar_t next_middle_time = 0;
            scalar_t nextStanceFinalTime = 0;
            if (swingFinalIndex < modeSequence.size() - 1) // 步态是切换的，且步态在列表内
            {
              const int nextStanceFinalIndex = finalTimesIndices[j][swingFinalIndex + 1]; // 下一次支撑点的结束相位为当前摆动相位结束的下一步
              nextStanceFinalTime = eventTimes[nextStanceFinalIndex];                     // 下一次支撑结束时间点（也是下一次摆动开始的时间点）
              next_middle_time = (swingFinalTime + nextStanceFinalTime) / 2;              // 中间时间 摆动结束的时刻(下一次支撑点开始时刻)到下一次支撑点结束时刻的中间时刻(为了计算落足点)
            }
            else
            {
              next_middle_time = swingFinalTime;
            }

            if(!start_elevation_map_){  //未启动感控
              vector_t next_middle_body_pos = targetTrajectories.getDesiredState(next_middle_time).segment<6>(6); // 参考轨迹中间时间点机体位置
              // 感控
              next_middle_body_pos_seq_[j].push_back(next_middle_body_pos);

              vector_t current_body_pos = targetTrajectories.getDesiredState(initTime).segment<6>(6); // 参考轨迹当前时间下机体位置
              vector_t current_body_vel = targetTrajectories.stateTrajectory[0].segment<3>(0);        // 参考轨迹起点速度(给定速度)

              next_stance_position[j] = calNextFootPos(j, initTime, swingFinalTime, next_middle_time, next_middle_body_pos,
                                                       current_body_pos, current_body_vel); // 计算落足点


              nextStandProjections_[j].clear();
              middleTimes_[j].clear();
              convexPolygonsInCurrentStand_[j].clear();
              StandProjections_[j].clear();
              initStandFinalTime_[j] = 0;
            }
            //*************************************感控*************************************
            else{
              
              //如果脚尖和脚跟所在的平面是不同的，则落足点仍为当前点(原地踏步)
              if(j == 0 || j == 1){
                // if(nextStandProjections_[j][p].regionPtr == nextStandProjections_[j+2][p].regionPtr){
                //   next_stance_position[j] = nextStandProjections_[j][p].positionInWorld;  //落足点更新
                // }
                // else{
                //   // 如果原地踏步，则当前的平面应该为上一次平面
                //   next_stance_position[j] = last_stance_position[j];
                //   auto penaltyFunction = [](const vector3_t & /*projectedPoint*/)
                //   { return 0.0; }; // 投影代价用的罚函数
                //   // 找到标准落脚点在所有分割平面的投影点，选取最优的投影点(x,y)，并将给投影点表示为世界坐标系(x,y,z)，作为真正的的落脚点
                //   auto projection = getBestPlanarRegionAtPositionInWorld(next_stance_position[j], planarTerrain_.planarRegions, penaltyFunction); // 仍然用原来的落足点计算投影点
                //   nextStandProjections_[j][p] = projection;
                //   next_stance_position[j] = projection.positionInWorld;
                // }
                //如果脚尖的点在脚跟的凸平面内
                if (convex_plane_decomposition::isInside(nextStandProjections_[j][p].positionInTerrainFrame, convexPolygonsInNextStand_[j + 2][p])){
                  next_stance_position[j] = nextStandProjections_[j][p].positionInWorld; // 落足点更新
                  scalar_t growthFactor = 1.05;
                  // 从落脚点为中心，在落脚点的平面上扩展出一个凸平面
                  const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                      nextStandProjections_[j][p].regionPtr->boundaryWithInset.boundary, nextStandProjections_[j][p].positionInTerrainFrame, numVertices_, growthFactor);

                  convexPolygonsInNextStand_[j][p] = convexRegion;
                  feet_front_[j][p] = true;
                }
                else{
                  next_stance_position[j] = last_stance_position[j];
                  auto penaltyFunction = [](const vector3_t & /*projectedPoint*/)
                  { return 0.0; }; // 投影代价用的罚函数
                  // 找到标准落脚点在所有分割平面的投影点，选取最优的投影点(x,y)，并将给投影点表示为世界坐标系(x,y,z)，作为真正的的落脚点
                  auto projection = getBestPlanarRegionAtPositionInWorld(next_stance_position[j], planarTerrain_.planarRegions, penaltyFunction); // 仍然用原来的落足点计算投影点
                  nextStandProjections_[j][p] = projection;
                  next_stance_position[j] = projection.positionInWorld;

                  scalar_t growthFactor = 1.05;
                  // 从落脚点为中心，在落脚点的平面上扩展出一个凸平面
                  const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                      projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numVertices_, growthFactor);

                  convexPolygonsInNextStand_[j][p] = convexRegion;
                  feet_front_[j][p] = false;
                }
              }
              else if(j == 2 || j == 3){
                // if(nextStandProjections_[j][p].regionPtr == nextStandProjections_[j-2][p].regionPtr){
                //   next_stance_position[j] = nextStandProjections_[j][p].positionInWorld; // 落足点更新
                // }
                // else{
                //   // 如果原地踏步，则当前的平面应该为上一次平面
                //   next_stance_position[j] = last_stance_position[j];
                //   auto penaltyFunction = [](const vector3_t & /*projectedPoint*/)
                //   { return 0.0; }; // 投影代价用的罚函数
                //   // 找到标准落脚点在所有分割平面的投影点，选取最优的投影点(x,y)，并将给投影点表示为世界坐标系(x,y,z)，作为真正的的落脚点
                //   auto projection = getBestPlanarRegionAtPositionInWorld(next_stance_position[j], planarTerrain_.planarRegions, penaltyFunction); //仍然用原来的落足点计算投影点
                //   nextStandProjections_[j][p] = projection;
                //   next_stance_position[j] = projection.positionInWorld;
                // }
                if(feet_front_[j-2][p]){  //说明脚尖与脚跟同处一个凸平面
                  next_stance_position[j] = nextStandProjections_[j][p].positionInWorld; // 落足点更新
                }
                else{
                  next_stance_position[j] = last_stance_position[j];
                  auto penaltyFunction = [](const vector3_t & /*projectedPoint*/)
                  { return 0.0; }; // 投影代价用的罚函数
                  // 找到标准落脚点在所有分割平面的投影点，选取最优的投影点(x,y)，并将给投影点表示为世界坐标系(x,y,z)，作为真正的的落脚点
                  auto projection = getBestPlanarRegionAtPositionInWorld(next_stance_position[j], planarTerrain_.planarRegions, penaltyFunction); // 仍然用原来的落足点计算投影点
                  nextStandProjections_[j][p] = projection;
                  next_stance_position[j] = projection.positionInWorld;

                  scalar_t growthFactor = 1.05;
                  // 从落脚点为中心，在落脚点的平面上扩展出一个凸平面
                  const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                      projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numVertices_, growthFactor);

                  convexPolygonsInNextStand_[j][p] = convexRegion;
                }
              }

              scalar_t growthFactor = 1.05;
              // 从落脚点为中心，在落脚点的平面上扩展出一个凸平面
              const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                  nextStandProjections_[j][p].regionPtr->boundaryWithInset.boundary, nextStandProjections_[j][p].positionInTerrainFrame, numVertices_, growthFactor);

              convexPolygonsInNextStand_[j][p] = convexRegion;

              // std::cout << "**************" << j << "**************" << std::endl;
              // std::cout << "**************" << p << "**************" << std::endl;
              // std::cout << next_stance_position[j] << std::endl;

              if (eesContactFlagStocks[j][p - 1] && ((p - 1) >= 0))
              { // 上一个步态为支撑
                convexPolygonsInCurrentStand_[j][p - 1].clear();
                convexPolygonsInCurrentStand_[j][p - 1] = convexPolygonsInNextStand_[j][p];
                StandProjections_[j][p - 1] = nextStandProjections_[j][p];
              }
              if (eesContactFlagStocks[j][p + 1] && ((p + 1) < modeSequence.size()))
              { // 下一个步态为支撑
                convexPolygonsInCurrentStand_[j][p + 1] = convexPolygonsInNextStand_[j][p];
                StandProjections_[j][p + 1] = nextStandProjections_[j][p];
              }
              convexPolygonsInCurrentStand_[j][p].clear();
              StandProjections_[j][p].regionPtr = nullptr;

              middleTimes_[j].push_back(next_middle_time);
            }
            //*************************************感控*************************************
            last_final_idx[j] = swingFinalIndex; // 更新上一次结束索引，防止重复计算落足点（摆动未结束时可能会再触发这个过程）
          }

          genSwingTrajs(j, initTime, swingStartTime, swingFinalTime, last_stance_position[j], next_stance_position[j]); // 得到摆动插值点
        }
        else // 支撑腿
        {
          const int stanceStartIndex = startTimesIndices[j][p]; // 支撑开始的相位
          const int stanceFinalIndex = finalTimesIndices[j][p]; // 支撑结束的相位

          const scalar_t stanceStartTime = eventTimes[stanceStartIndex];
          const scalar_t stanceFinalTime = eventTimes[stanceFinalIndex];
          const scalar_t time_length = stanceFinalTime - stanceStartTime;

          startStopTime_[j].push_back({stanceStartTime, stanceFinalTime});
          const std::vector<CubicSpline::Node> x_nodes{                                                                    // 构造足端支撑点（当前站立的点）
                                                       CubicSpline::Node{stanceStartTime, next_stance_position[j].x(), 0}, // 支撑时足端速度为0，且没有滑动，所以开始和结束时的位置相同
                                                       CubicSpline::Node{stanceFinalTime, next_stance_position[j].x(), 0}};
          feetXTrajs_[j].emplace_back(x_nodes); // 把点放入到轨迹

          const std::vector<CubicSpline::Node> y_nodes{
              CubicSpline::Node{stanceStartTime, next_stance_position[j].y(), 0},
              CubicSpline::Node{stanceFinalTime, next_stance_position[j].y(), 0}};
          feetYTrajs_[j].emplace_back(y_nodes);
          const std::vector<CubicSpline::Node> z_nodes{
              CubicSpline::Node{stanceStartTime, next_stance_position[j].z(), 0},
              CubicSpline::Node{stanceFinalTime, next_stance_position[j].z(), 0}};
          feetZTrajs_[j].emplace_back(z_nodes);

          if (initTime < stanceFinalTime && stanceFinalIndex > last_stand_final_idx[j])
          {
            // 感控
            if (start_elevation_map_)
            {
              if (eesContactFlagStocks[j][p + 1] && ((p + 1) < modeSequence.size()))
              { // 下一个相位为支撑相
                if (p == 0)
                { // 初始时
                  auto penaltyFunction = [](const vector3_t & /*projectedPoint*/)
                  { return 0.0; }; // 投影代价用的罚函数
                  // 找到标准落脚点在所有分割平面的投影点，选取最优的投影点(x,y)，并将给投影点表示为世界坐标系(x,y,z)，作为真正的的落脚点
                  const auto projection = getBestPlanarRegionAtPositionInWorld(next_stance_position[j], planarTerrain_.planarRegions, penaltyFunction);
                  scalar_t growthFactor = 1.05;
                  const auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                      projection.regionPtr->boundaryWithInset.boundary, projection.positionInTerrainFrame, numVertices_, growthFactor); // 从落脚点为中心，在落脚点的平面上扩展出一个凸平面

                  convexPolygonsInCurrentStand_[j][p] = convexRegion;
                  convexPolygonsInCurrentStand_[j][p + 1] = convexPolygonsInCurrentStand_[j][p];

                  StandProjections_[j][p] = projection;
                  StandProjections_[j][p + 1] = StandProjections_[j][p];
                }
                else
                {
                  convexPolygonsInCurrentStand_[j][p + 1] = convexPolygonsInCurrentStand_[j][p];
                  StandProjections_[j][p + 1] = StandProjections_[j][p];
                }
                if ((!eesContactFlagStocks[j][p - 1]) && ((p - 1) >= 0))
                { // 上一个步态为摆动
                  convexPolygonsInCurrentStand_[j][p - 1].clear();
                  StandProjections_[j][p - 1].regionPtr = nullptr;
                }
              }

              if (stanceStartTime < initTime && initTime < stanceFinalTime)
              {
                initStandFinalTime_[j] = stanceFinalTime; // 当前时间如果是支撑相，记录这个时间，作为现实的支撑结束时间
              }
            }
            else
            {
              nextStandProjections_[j].clear();
              middleTimes_[j].clear();
              convexPolygonsInCurrentStand_[j].clear();
              StandProjections_[j].clear();
              initStandFinalTime_[j] = 0;
            }
            last_stand_final_idx[j] = stanceFinalIndex;
          }
        }
      }
      // std::cout << "*****************" << j << "*****************"  << std::endl;
      // std::cout << next_stance_position[j] << std::endl;
      feetTrajsEvents_[j] = eventTimes; // 每个足端点的步态切换时间表
  }


  feetXTrajsBuf_.setBuffer(feetXTrajs_);    //把足端轨迹存到缓冲区
  feetYTrajsBuf_.setBuffer(feetYTrajs_);
  feetZTrajsBuf_.setBuffer(feetZTrajs_);
  StartStopTimeBuf_.setBuffer(startStopTime_);
  footTrajsEventsBuf_.setBuffer(feetTrajsEvents_[0]);
}

// ref: Highly Dynamic Quadruped Locomotion via Whole-Body Impulse Control and Model Predictive Control   MIT机器狗
vector3_t SwingTrajectoryPlanner::calNextFootPos(int feet, scalar_t current_time, scalar_t stop_time,
                                                 scalar_t next_middle_time, const vector_t& next_middle_body_pos,
                                                 const vector_t& current_body_pos, const vector3_t& current_body_vel)    //计算下一次支撑点的位置
{
  vector3_t next_stance_position;   //落足点
  vector3_t angular = next_middle_body_pos.tail(3);   //机体姿态
  vector3_t roted_bias = getRotationMatrixFromZyxEulerAngles(angular) * feet_bias_[feet]; // 中间时间点时足端点在世界世界坐标偏置
  vector3_t current_angular = current_body_pos.tail(3);   //参考轨迹起点机体位置
  auto current_rot = getRotationMatrixFromZyxEulerAngles(current_angular);
  const vector3_t& vel_cmd_linear = current_rot * body_vel_cmd_.head(3);    //速度给定 v_cmd
  const vector3_t& vel_cmd_angular = current_rot * body_vel_cmd_.tail(3);

  vector3_t current_body_vel_tmp = current_body_vel;    
  current_body_vel_tmp(2) = 0;
  const vector3_t &vel_linear = current_body_vel_tmp; // 参考轨迹当前时间下机体速度 v_current

  const scalar_t k = 0.03;
  vector3_t p_shoulder = (stop_time - current_time) * (0.5 * vel_linear + 0.5 * vel_cmd_linear) + roted_bias;   //v*t + p_bias  用速度给定做一些补偿
  vector3_t p_symmetry = (next_middle_time - stop_time) * vel_linear + k * (vel_linear - vel_cmd_linear); //tsatance/2 * v_current + k*(v_current - v_cmd)
  vector3_t p_centrifugal = 0.5 * sqrt(current_body_pos(2) / 9.81) * vel_linear.cross(vel_cmd_angular); 
  next_stance_position = current_body_pos.head(3) + p_shoulder + p_symmetry + p_centrifugal;
  next_stance_position.z() = config_.next_position_z;
  return std::move(next_stance_position);
}

vector3_t SwingTrajectoryPlanner::calBiasPos2d(vector3_t bias,const vector3_t &source_pos, const vector_t &next_middle_body_pos){
  vector3_t fix_stance_position;
  scalar_t z_angle = next_middle_body_pos[3]; // 机体姿态
  matrix3_t Rz;
  Rz << cos(z_angle), -sin(z_angle), 0, // clang-format off
         sin(z_angle), cos(z_angle), 0,
         0, 0, 1; 
  fix_stance_position = source_pos + Rz * bias;
  return fix_stance_position;
}

void SwingTrajectoryPlanner::genSwingTrajs(int feet, scalar_t current_time, scalar_t start_time, scalar_t stop_time,
                                           const vector3_t& start_pos, const vector3_t& stop_pos)   //规划轨迹
{ //足端点位置规划，足端点位置期望
  scalar_t xy_a1 = 0.417;   //时间占比    控制加速
  scalar_t xy_l1 = 0.650;   //位置占比    
  scalar_t xy_k1 = 1.770;   //速度占比    (切线)
  //一个摆动步态对应的插值节点，x节点有3个插值点，即2个插值段
  const std::vector<CubicSpline::Node> x_nodes{     //起点速度和终点速度都为0
    CubicSpline::Node{ start_time, start_pos.x(), 0 },
    CubicSpline::Node{ (1 - xy_a1) * start_time + xy_a1 * stop_time, (1 - xy_l1) * start_pos.x() + xy_l1 * stop_pos.x(),
                       xy_k1 * (stop_pos.x() - start_pos.x()) / (stop_time - start_time) },   //摆动点可以自定义？ 在xy_a1之内加速到xy_k1的速度，且到达xy_l1的位置
    CubicSpline::Node{ stop_time, stop_pos.x(), 0 }
  };
  feetXTrajs_[feet].emplace_back(x_nodes); // 会把每一段插值多项式系数也计算出来(emplace_back在插入容器时同时构造对象，调用构造函数MultiCubicSpline)

  const std::vector<CubicSpline::Node> y_nodes{
    CubicSpline::Node{ start_time, start_pos.y(), 0 },
    CubicSpline::Node{ (1 - xy_a1) * start_time + xy_a1 * stop_time, (1 - xy_l1) * start_pos.y() + xy_l1 * stop_pos.y(),
                       xy_k1 * (stop_pos.y() - start_pos.y()) / (stop_time - start_time) },
    CubicSpline::Node{ stop_time, stop_pos.y(), 0 }
  };
  feetYTrajs_[feet].emplace_back(y_nodes);

  const scalar_t scaling = swingTrajectoryScaling(start_time, stop_time, config_.swingTimeScale);
  const scalar_t max_z = std::max(start_pos.z(), stop_pos.z()) + scaling * config_.swingHeight;   //抬腿最高点
  scalar_t z_a1 = 0.251;
  scalar_t z_l1 = 0.749;
  scalar_t z_k1 = 1.338;

  scalar_t z_a2 = 0.630;
  scalar_t z_l2 = 0.570;
  scalar_t z_k2 = 1.633;

  scalar_t z_k3 = 0.000;
  //z方向分成4个点
  const std::vector<CubicSpline::Node> z_nodes{   //z方向落脚速度不为零且大小为负值
    CubicSpline::Node{ start_time, start_pos.z(), 0 },
    CubicSpline::Node{ (1 - z_a1) * start_time + z_a1 * stop_time, z_l1 * max_z,
                       z_k1 * (z_l1 * (max_z - start_pos.z())) / ((z_a1) * (stop_time - start_time)) },
    CubicSpline::Node{ (1 - z_a2) * start_time + z_a2 * stop_time, z_l2 * max_z + (1 - z_l2) * stop_pos.z(),
                       z_k2 * z_l2 * (stop_pos.z() - max_z) / ((1 - z_a2) * (stop_time - start_time)) },
    CubicSpline::Node{ stop_time, stop_pos.z(),
                       z_k3 * z_l2 * (stop_pos.z() - max_z) / ((1 - z_a2) * (stop_time - start_time)) }
  };
  feetZTrajs_[feet].emplace_back(z_nodes);
}

convex_plane_decomposition::CgalPolygon2d SwingTrajectoryPlanner::getConvexPolygon(size_t leg, scalar_t time) const
{
  const auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  //找到当前时间的索引
  return convexPolygonsInCurrentStand_[leg][index];
}

convex_plane_decomposition::PlanarTerrainProjection SwingTrajectoryPlanner::getProjection(size_t leg, scalar_t time) const
{
  const auto index = lookup::findIndexInTimeArray(feetTrajsEvents_[leg], time);
  return StandProjections_[leg][index];
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<std::vector<int>, std::vector<int>>
SwingTrajectoryPlanner::updateFootSchedule(const std::vector<bool>& contactFlagStock)
{
  const size_t numPhases = contactFlagStock.size();   //根据步态列表的步态个数确定，得到某一个接触点在这个列表下的所有状态

  std::vector<int> startTimeIndexStock(numPhases, 0);   
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing and stance feet
  for (size_t i = 0; i < numPhases; i++)
  {
    std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);    //每一相位的接触点开始相位和结束相位
  }
  return { startTimeIndexStock, finalTimeIndexStock };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<bool>>
SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const
{
  const size_t numPhases = phaseIDsStock.size();    //步态序列大小  modeSeq：STANCE L R L R L R L R L R STANCE 12个步态相位

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++)
  {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);    //根据步态序列，查找对应接触点情况
    for (size_t j = 0; j < numFeet_; j++)
    {
      contactFlagStock[j][i] = contactFlag[j];    //每个相位对应的接触点情况，按列排列，列表示步态类型，行表示各个接触点在某个相位的接触点情况
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock)  //返回接触点在步态列表里某一状态的开始和结束相位
{
  const size_t numPhases = contactFlagStock.size();

  // find the starting time
  int startTimesIndex = 0;
  for (int ip = index - 1; ip >= 0; ip--)
  {
    if (contactFlagStock[ip] != contactFlagStock[index])    //当前相位接触状态不等于上一步的接触状态，则此时相位的开始索引即上一步相位
    {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 2;
  for (size_t ip = index + 1; ip < numPhases; ip++)
  {
    if (contactFlagStock[ip] != contactFlagStock[index])  //当前相位接触状态不等于下一步的接触状态，则此时相位的结束索引即当前相位索引
    {
      finalTimesIndex = ip - 1;
      break;
    }
  }
  // 比如trot步态下，左脚和右脚切换运行，则每个相位的状态不同，则开始为上一步相位(结束)，结束为当前相位；如果是stance步态，每个相位状态都是相同的，开始的时间从第一个相位开始，到最后一个相位结束
  return { startTimesIndex, finalTimesIndex };  
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock)
{
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0)
  {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++)
    {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) +
                             " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1)
  {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++)
    {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) +
                             " is not defined.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::array<vector_t, 6>> SwingTrajectoryPlanner::threadSaftyGetPosVel(const vector_t& time_sample)
{
  feetXTrajsBuf_.updateFromBuffer();
  feetYTrajsBuf_.updateFromBuffer();
  feetZTrajsBuf_.updateFromBuffer();
  footTrajsEventsBuf_.updateFromBuffer();
  const auto& Xs = feetXTrajsBuf_.get();
  const auto& Ys = feetYTrajsBuf_.get();
  const auto& Zs = feetZTrajsBuf_.get();
  const auto& footTrajsEvents_ = footTrajsEventsBuf_.get();

  size_t sample_size = time_sample.size();

  feet_array_t<std::array<vector_t, 6>> feet_pos_vel;
  for (int leg = 0; leg < numFeet_; leg++)
  {
    for (int j = 0; j < feet_pos_vel[leg].size(); j++)
    {
      feet_pos_vel[leg][j].resize(sample_size);
    }
  }

  for (size_t i = 0; i < sample_size; i++)
  {
    auto index = lookup::findIndexInTimeArray(footTrajsEvents_, time_sample(i));
    index = std::min((int)(footTrajsEvents_.size() - 1), index);

    for (int leg = 0; leg < numFeet_; leg++)
    {
      feet_pos_vel[leg][0][i] = Xs[leg][index].position(time_sample(i));
      feet_pos_vel[leg][1][i] = Ys[leg][index].position(time_sample(i));
      feet_pos_vel[leg][2][i] = Zs[leg][index].position(time_sample(i));

      feet_pos_vel[leg][3][i] = Xs[leg][index].velocity(time_sample(i));
      feet_pos_vel[leg][4][i] = Ys[leg][index].velocity(time_sample(i));
      feet_pos_vel[leg][5][i] = Zs[leg][index].velocity(time_sample(i));
    }
  }
  return std::move(feet_pos_vel);
}

feet_array_t<std::array<scalar_t, 2>> SwingTrajectoryPlanner::threadSaftyGetStartStopTime(scalar_t time)
{
  feet_array_t<std::array<scalar_t, 2>> startStopTime4legs;
  StartStopTimeBuf_.updateFromBuffer();
  footTrajsEventsBuf_.updateFromBuffer();
  const auto& Ts = StartStopTimeBuf_.get();   //包含每个足端点摆动轨迹的开始时间和结束时间
  const auto& footTrajsEvents_ = footTrajsEventsBuf_.get(); //包含足端摆动轨迹事件
  auto index = lookup::findIndexInTimeArray(footTrajsEvents_, time);  //根据当前的时间寻找对应的足端时间节点
  index = std::min((int)(footTrajsEvents_.size() - 1), index);  //防止越界
  for (int leg = 0; leg < numFeet_; leg++)
  {
    if (Ts[leg].size())   //如Ts的数据存在，则使用Ts包含的时间
    {
      startStopTime4legs[leg] = Ts[leg][index];
    }
    else    //否则使用当前的系统时间
    {
      startStopTime4legs[leg] = std::array<scalar_t, 2>{ time, time };
    }
  }

  return std::move(startStopTime4legs);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName, const std::string& fieldName,
                                                           bool verbose)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose)
  {
    std::cerr << "\n #### Swing Trajectory Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  SwingTrajectoryPlanner::Config config;
  const std::string prefix = fieldName + ".";

  loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
  loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_x1, prefix + "feet_bias_x1", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_x2, prefix + "feet_bias_x2", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_y, prefix + "feet_bias_y", verbose);
  loadData::loadPtreeValue(pt, config.feet_bias_z, prefix + "feet_bias_z", verbose);
  loadData::loadPtreeValue(pt, config.next_position_z, prefix + "next_position_z", verbose);

  if (verbose)
  {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return config;
}

}  // namespace legged_robot
}  // namespace ocs2
