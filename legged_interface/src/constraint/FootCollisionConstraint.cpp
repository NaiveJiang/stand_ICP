//
// Created by qiayuan on 23-1-26.
//
#include "legged_interface/constraint/FootCollisionConstraint.h"

namespace legged {
FootCollisionConstraint::FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                 const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                 std::shared_ptr<grid_map::SignedDistanceField> sdfPtr, size_t contactPointIndex,
                                                 scalar_t clearance)      //摆腿的足端碰撞约束，防止摆腿的时候足端碰撞到台阶   clearance为碰撞sdf球的半径，保证这个足端点在这个半径范围内无碰撞
    : StateConstraint(ConstraintOrder::Linear),   //线性约束
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      sdfPtr_(std::move(sdfPtr)),
      contactPointIndex_(contactPointIndex),
      clearance_(clearance) {}

FootCollisionConstraint::FootCollisionConstraint(const FootCollisionConstraint& rhs)
    : StateConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      sdfPtr_(rhs.sdfPtr_),
      contactPointIndex_(rhs.contactPointIndex_),
      clearance_(rhs.clearance_) {}

bool FootCollisionConstraint::isActive(scalar_t time) const {   //只有摆动的过程才需要这个碰撞约束，要保证这个约束是严格处于摆动状态下的，要避免刚抬脚和刚落脚的时候
  scalar_t offset = 0.05;
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_] &&         //确定当前是摆动腿
         !referenceManagerPtr_->getContactFlags(time + 0.5 * offset)[contactPointIndex_] &&   //如果一定时间后仍为摆动相，否则可能是刚放下脚的时候，sdf球体处于碰撞态
         !referenceManagerPtr_->getContactFlags(time - offset)[contactPointIndex_]; //一定时间之前也是摆动相，否则可能当前是刚抬脚的时候，sdf球体处于碰撞态    
}

vector_t FootCollisionConstraint::getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const {   //需要得到当前状态下的约束函数的值来进行一阶近似
  vector_t value(1);
  value(0) = sdfPtr_->value(grid_map::Position3(endEffectorKinematicsPtr_->getPosition(state).front())) - clearance_; //约束函数：保证SDF球-0.03的球仍然大于等于0，即足端点到碰撞体的距离至少有0.03m(3cm)
  return value;
}

VectorFunctionLinearApproximation FootCollisionConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                  const PreComputation& preComp) const {    //线性化约束
  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(1, state.size(), 0);
  approx.f = getValue(time, state, preComp);    //当前状态下的约束函数值
  approx.dfdx = sdfPtr_->derivative(grid_map::Position3(endEffectorKinematicsPtr_->getPosition(state).front())).transpose() *
                endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front().dfdx;
  return approx;
}

}  // namespace legged
