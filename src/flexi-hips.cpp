#include "flex-joints/flexi-hips.hpp"

#include <iostream>

namespace flex {

Flex::Flex() {
  // default settigs
}

Flex::Flex(const FlexSettings &settings) { initialize(settings); }

void Flex::initialize(const FlexSettings &settings) {
  settings_ = settings;
  MA_samples_ = (int)round(settings.MA_duration / settings.dt);
}

const eVector2 &Flex::computeDeflection(const eArray2 &torques,
                                        const eArray2 &delta0,
                                        const eArray2 &stiffness,
                                        const eArray2 &damping,
                                        const double dt) {
  /** Computes the deflection expected for certain flexing torque
   * according to:
   *
   *   delta = (delta0 * damping - torques * dt) * inverse(stiffness * dt +
   * damping)
   */
  temp_damping_ << delta0 * damping;
  temp_actuation_ << torques * dt;
  temp_full_torque_ << temp_damping_ - temp_actuation_;
  temp_stiff_ << stiffness * dt;
  temp_equiv_stiff_ << temp_stiff_ + damping;
  temp_compliance_ << inverse(temp_equiv_stiff_);
  computed_deflection_ = (temp_full_torque_ * temp_compliance_).matrix();

  return computed_deflection_;
}

const eVector3 &Flex::equivalentAngles(const eMatrixRot &fullRotation) {
  /**
   * Computes three angles with the order "z-x-y" such that their
   * combined rotation is equivalent to the "fullRotation".
   */
  qz_ = atan2(-fullRotation(0, 1), fullRotation(1, 1));
  resulting_angles_ << qz_,
      atan2(fullRotation(2, 1),
            -fullRotation(0, 1) * sin(qz_) + fullRotation(1, 1) * cos(qz_)),
      atan2(-fullRotation(2, 0), fullRotation(2, 2));
  return resulting_angles_;
}

void Flex::correctHip(const eVector2 &delta, const eVector2 &deltaDot,
                      eVectorX &q, eVectorX &dq,
                      const Eigen::Array3i &hipIndices) {
  rotationA_ = Eigen::AngleAxisd(delta(0), eVector3::UnitY());
  rotationB_ = rotationA_ * Eigen::AngleAxisd(delta(1), eVector3::UnitX());
  rotationC_ =
      rotationB_ * Eigen::AngleAxisd(q(hipIndices(0)), eVector3::UnitZ());
  rotationD_ =
      rotationC_ * Eigen::AngleAxisd(q(hipIndices(1)), eVector3::UnitX());
  rotationE_ =
      rotationD_ * Eigen::AngleAxisd(q(hipIndices(2)), eVector3::UnitY());

  q.segment(hipIndices(0), 3) = equivalentAngles(rotationE_);

  flexRateY_ << 0, deltaDot(0), 0;
  flexRateX_ = rotationA_.col(0) * deltaDot(1);
  dqZ_ = rotationB_.col(2) * dq(hipIndices(0));
  dqX_ = rotationC_.col(0) * dq(hipIndices(1));
  dqY_ = rotationD_.col(1) * dq(hipIndices(2));

  legAngularVelocity_ = flexRateY_ + flexRateX_ + dqZ_ + dqX_ + dqY_;

  rigidRotC_ = Eigen::AngleAxisd(q(hipIndices(0)), eVector3::UnitZ());
  rigidRotD_ =
      rigidRotC_ * Eigen::AngleAxisd(q(hipIndices(1)), eVector3::UnitX());

  M_ << eVector3::UnitZ(), rigidRotC_.col(0), rigidRotD_.col(1);
  dq.segment(hipIndices(0), 3) = M_.inverse() * legAngularVelocity_;
}

void Flex::correctDeflections(const eVector2 &leftFlexingTorque,
                              const eVector2 &rightFlexingTorque, eVectorX &q,
                              eVectorX &dq) {
  /**
   * Arguments:
   *
   * leftFlexTorque and rightFlexingTorque are composed as {pitch_torque,
   * roll_torque}
   *
   * q and dq are the robot posture and velocities without the freeFlyer part.
   */

  leftFlex_ << computeDeflection(leftFlexingTorque, leftFlex0_,
                                 settings_.left_stiffness,
                                 settings_.left_damping, settings_.dt);
  rightFlex_ << computeDeflection(rightFlexingTorque, rightFlex0_,
                                  settings_.right_stiffness,
                                  settings_.right_damping, settings_.dt);

  if (settings_.filtered) {
    leftFlexRate_ = movingAverage((leftFlex_ - leftFlex0_) / settings_.dt,
                                  queue_LH_, summation_LH_);
    rightFlexRate_ = movingAverage((rightFlex_ - rightFlex0_) / settings_.dt,
                                   queue_RH_, summation_RH_);
  } else {
    leftFlexRate_ = (leftFlex_ - leftFlex0_) / settings_.dt;
    rightFlexRate_ = (rightFlex_ - rightFlex0_) / settings_.dt;
  }

  leftFlex0_ = leftFlex_;
  rightFlex0_ = rightFlex_;

  correctHip(leftFlex_, leftFlexRate_, q, dq, settings_.left_hip_indices);
  correctHip(rightFlex_, rightFlexRate_, q, dq, settings_.right_hip_indices);
}

const eVector2 &Flex::estimateFlexingTorque(const eVector3 &hipPos,
                                            const eVector3 &jointTorque){

  flexRotation_ = Eigen::Rotation2Dd(-hipPos(0));
  flexRotation_.col(1) *= cos(hipPos(1));
  getFlexing_ = xy_to_yx * flexRotation_;

  flexingTorque_ = getFlexing_ * jointTorque.tail<2>();
  return flexingTorque_;
}

const eVector2 &Flex::estimateFlexingTorque(const eVector3 &hipPos,
                                            const eVector3 &jointTorque,
                                            const eVector2 &delta0,
                                            const eVector3 &jointForce){
                                           
  deformRotation_ = Eigen::AngleAxisd(delta0(0), eVector3::UnitY()) * 
                    Eigen::AngleAxisd(delta0(1), eVector3::UnitX());
  currentFlexToJoint_ = deformRotation_ * settings_.flexToJoint;
  r_x_F_ = currentFlexToJoint_.cross(jointForce);

  flexingTorque_ = estimateFlexingTorque(hipPos, jointTorque) + r_x_F_.head<2>(); // check sign of rxF
  return flexingTorque_;
}

void Flex::correctEstimatedDeflections(const eVectorX &desiredTorque,
                                       eVectorX &q, eVectorX &dq) {
  /**
   * Arguments:
   *
   * desired torque are the values commanded by the controller.
   *
   * q and dq are the robot posture and velocities without the freeFlyer part.
   *
   */
  
  flexingLeftTorque_ = estimateFlexingTorque(q.segment(settings_.left_hip_indices(0), 3),
                                             desiredTorque.segment(settings_.left_hip_indices(0), 3));
  flexingRightTorque_ = estimateFlexingTorque(q.segment(settings_.right_hip_indices(0), 3),
                                              desiredTorque.segment(settings_.right_hip_indices(0), 3));                                          

  correctDeflections(flexingLeftTorque_, flexingRightTorque_, q, dq);
}

/// @todo: Implement methods for a better estimation of the flexing torque. i.e.
// including the gravity. Giulio made a function for such estimation.
// Alternatively, a better estimation can be obtained from RNEA.

const eArray2 &Flex::movingAverage(const eArray2 &x, std::deque<eArray2> &queue,
                                   eArray2 &summation) {
  queue.push_back(x);
  queueSize_ = queue.size();

  if (queueSize_ > MA_samples_) {
    summation -= queue[0];
    queue.pop_front();
    queueSize_--;
  }

  summation += queue[queueSize_ - 1];
  average_ = summation / queueSize_;
  return average_;
}

void Flex::reset() {
  leftFlex0_ = eVector2::Zero();
  rightFlex0_ = eVector2::Zero();
  summation_LH_ = eArray2::Zero();
  summation_RH_ = eArray2::Zero();
  queue_LH_.clear();
  queue_RH_.clear();
}

}  // namespace flex
