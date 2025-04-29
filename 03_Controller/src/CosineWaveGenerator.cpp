#include "CosineWaveGenerator.h"
#include <array>
#include <cmath>

#include "Eigen/Dense"

/**
Cosine-wave trajectory,
Position follows q_i(t) = q_i(0) + A_i * (1 - cos(\omega_i * t)
Velocity follows \dot{q}_i(t) = -A_i * \omega_i * sin(\omega_i * t)
Acceleration follows \ddot{q}_i(t) = - A_i * \omega_i^2 * cos(\omega_i * t)
**/
std::array<double, 7> CosineWaveMotionGenerator::CosineWavePosition(
    const double time, const std::array<double, 7> &Initial_Positions,
    const std::array<double, 7> &Amplitudes,
    const std::array<double, 7> &Omegas) {
  std::array<double, 7> position; // [rad]
  for (size_t i = 0; i < 7; i++) {
    position[i] =
        Initial_Positions[i] + Amplitudes[i] * (1 - cos(Omegas[i] * time));
  }
  return position;
}

std::array<double, 7> CosineWaveMotionGenerator::CosineWaveVelocity(
    const double time, const std::array<double, 7> &Amplitudes,
    const std::array<double, 7> &Omegas) {
  std::array<double, 7> velocity; // [rad/s]
  for (size_t i = 0; i < 7; i++) {
    velocity[i] = -Amplitudes[i] * Omegas[i] * sin(Omegas[i] * time);
  }
  return velocity;
}

std::array<double, 7> CosineWaveMotionGenerator::CosineWaveAcceleration(
    const double time, const std::array<double, 7> &Amplitudes,
    const std::array<double, 7> &Omegas) {
  std::array<double, 7> acceleration; // [rad/s^2]
  for (size_t i = 0; i < 7; i++) {
    acceleration[i] =
        -Amplitudes[i] * pow(Omegas[i], 2) * cos(Omegas[i] * time);
  }
  return acceleration;
}

CosineWaveMotionGenerator::CosineWaveMotionGenerator(
    const std::array<double, 7> &initial_joint_position,
    const std::array<double, 7> &A, const std::array<double, 7> &omega,
    const double &duration)
    : initial_joint_position_(initial_joint_position), A_(A), omega_(omega),
      duration_(duration) {
  initial_joint_position_ = initial_joint_position;
  A_ = A;
  omega_ = omega;
  duration_ = duration;
  time_ = 0.0;

  q_end_ =
      CosineWavePosition(duration_, initial_joint_position_, A_, omega_);
  dq_end_ = CosineWaveVelocity(duration_, A_, omega_);
  ddq_end_ = CosineWaveAcceleration(duration_, A_, omega_);
}

franka::JointPositions
CosineWaveMotionGenerator::operator()(const franka::RobotState &robot_state,
                                      franka::Duration period) {
  time_ += period.toSec();

  std::array<double, 7> joint_positions;
  bool motion_finished = (time_ >= duration_);

  joint_positions =
      CosineWavePosition(time_, initial_joint_position_, A_, omega_);

  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}

std::array<double, 7>
CosineWaveMotionGenerator::trajectory_position(const double &time) {
  return CosineWavePosition(time, initial_joint_position_, A_, omega_);
}

std::array<double, 7>
CosineWaveMotionGenerator::trajectory_velocity(const double &time) {
  return CosineWaveVelocity(time, A_, omega_);
}

std::array<double, 7>
CosineWaveMotionGenerator::trajectory_acceleration(const double &time) {
  return CosineWaveAcceleration(time, A_, omega_);
}

std::array<std::array<double, 7>, 3>
CosineWaveMotionGenerator::kinematics_end() {
  std::array<std::array<double, 7>, 3>
      kinematics_end; // array for q_end, dq_end, ddq_end
  kinematics_end[0] = q_end_;
  kinematics_end[1] = dq_end_;
  kinematics_end[2] = ddq_end_;
  return kinematics_end;
}