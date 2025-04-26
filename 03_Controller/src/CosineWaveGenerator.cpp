#include "CosineWaveGenerator.h"
#include <array>
#include <cmath>

#include "Eigen/Dense"

/**
Cosine-wave trajectory, q_i(t) = q_i(0) + A_i * (1 - cos(omega_i * t)
Velocity follows \dot{q}_i(t) = -A_i * omega_i * sin(omega_i * t)
**/
std::array<double, 7>
CosineWavePosition(const double time,
                   const std::array<double, 7> &Initial_Positions,
                   const std::array<double, 7> &Amplitudes,
                   const std::array<double, 7> &Omegas) {
  std::array<double, 7> position; // [rad]
  for (size_t i = 0; i < 7; i++) {
    position[i] =
        Initial_Positions[i] + Amplitudes[i] * (1 - cos(Omegas[i] * time));
  }
  return position;
}

std::array<double, 7>
CosineWaveVelocity(const double time, const std::array<double, 7> &Amplitudes,
                   const std::array<double, 7> &Omegas) {
  std::array<double, 7> velocity; // [rad/s]
  for (size_t i = 0; i < 7; i++) {
    velocity[i] = -Amplitudes[i] * Omegas[i] * sin(Omegas[i] * time);
  }
  return velocity;
}

std::array<double, 7>
CosineWaveAcceleration(const double time,
                       const std::array<double, 7> &Amplitudes,
                       const std::array<double, 7> &Omegas) {
  std::array<double, 7> acceleration; // [rad/s^2]
  for (size_t i = 0; i < 7; i++) {
    acceleration[i] =
        -Amplitudes[i] * pow(Omegas[i], 2) * cos(Omegas[i] * time);
  }
  return acceleration;
}

/**
Polynomial Trajectory
q (t) = a_0 + a_1*t + a_2*t^2 + a_3*t^3 + a_4*t^4 + a_5*t^5.
\dot{q} (t) = a_1 + 2*a_2*t + 3*a_3*t^2 + 4*a_4*t^3 + 5*a_5*t^4.
\ddot{q} (t) = 2*a_2 + 6*a_3*t + 12*a_4*t^2 + 20*a_5*t^3.
Solve linear equations A x = b, where A and b follows
A = [ 1, t_1, t_1^2, t_1^3, t_1^4, t_1^5,
0, 1, 2*t_1, 3*t_1^2, 4*t_1^3, 5*t_1^4,
0, 0, 2, 6*t_1, 12*t_1^2, 20*t_1^3,
1, t_2, t_2^2, t_2^3, t_2^4, t_2^5,
0, 1, 2*t_2, 3*t_2^2, 4*t_2^3, 5*t_2^4,
0, 0, 2, 6*t_2, 12*t_2^2, 20*t_2^3],
b = [ q_1, \dot{q}_1, \ddot{q}_1, q_2, \dot{q}_2, \ddot{q}_2 ]^T.
**/
std::array<double, 6> Polynomial_Coefficients(
    const double &time_start, const double &time_end,
    const double &position_start, const double &velocity_start,
    const double &acceleration_start, const double &position_end,
    const double &velocity_end, const double &acceleration_end) {
  Eigen::Matrix<double, 6, 6> A;
  A << 1, time_start, pow(time_start, 2), pow(time_start, 3),
      pow(time_start, 4), pow(time_start, 5), // 1st Row
      0, 1, 2 * time_start, 3 * pow(time_start, 2), 4 * pow(time_start, 3),
      5 * pow(time_start, 4), 0, // 2nd Row
      0, 2, 6 * time_start, 12 * pow(time_start, 2),
      20 * pow(time_start, 3), // 3rd Row
      1, time_end, pow(time_end, 2), pow(time_end, 3), pow(time_end, 4),
      pow(time_end, 5), // 4th Row
      0, 1, 2 * time_end, 3 * pow(time_end, 2), 4 * pow(time_end, 3),
      5 * pow(time_end, 4), // 5th Row
      0, 0, 2, 6 * time_end, 12 * pow(time_end, 2),
      20 * pow(time_end, 3); // 6th Row
  Eigen::Matrix<double, 6, 1> b;
  b << position_start, velocity_start, acceleration_start, position_end,
      velocity_end, acceleration_end;
  Eigen::Matrix<double, 6, 1> Coefficients = A.colPivHouseholderQr().solve(b);
  std::array<double, 6> coefficient_array;
  for (size_t i = 0; i < 6; i++) {
    coefficient_array[i] = Coefficients(i);
  }
  return coefficient_array;
}

std::array<double, 7> PolynomialPosition(
    const double &time,
    const std::array<std::array<double, 6>, 7> &trajectory_coefficients) {
  std::array<double, 7> position;
  for (size_t i = 0; i < 7; i++) {
    std::array<double, 6> coefficients = trajectory_coefficients[i];
    position[i] =
        coefficients[0] + coefficients[1] * time +
        coefficients[2] * pow(time, 2) + coefficients[3] * pow(time, 3) +
        coefficients[4] * pow(time, 4) + coefficients[5] * pow(time, 5);
  }
  return position;
}

CosineMotionGenerator::CosineMotionGenerator(
    const std::array<double, 7> &initial_joint_position,
    const std::array<double, 7> &A, const std::array<double, 7> &omega,
    const double &t_wave_end, const double &backing_time)
    : initial_joint_position_(initial_joint_position), A_(A), omega_(omega),
      t_wave_end_(t_wave_end), backing_time_(backing_time) {
  initial_joint_position_ = initial_joint_position;
  A_ = A;
  omega_ = omega;
  t_wave_end_ = t_wave_end;
  backing_time_ = backing_time;
  time_ = 0.0;

  q_wave_end_ =
      CosineWavePosition(t_wave_end_, initial_joint_position_, A_, omega_);
  dq_wave_end_ = CosineWaveVelocity(t_wave_end_, A_, omega_);
  ddq_wave_end_ = CosineWaveAcceleration(t_wave_end_, A_, omega_);
  for (size_t i = 0; i < 7; ++i) {
    backing_trajectory_coefficients_[i] = Polynomial_Coefficients(
        t_wave_end_, t_wave_end_ + backing_time_, q_wave_end_[i],
        dq_wave_end_[i], ddq_wave_end_[i], initial_joint_position_[i], 0, 0);
  }
}

franka::JointPositions
CosineMotionGenerator::operator()(const franka::RobotState &robot_state,
                                  franka::Duration period) {
  time_ += period.toSec();

  std::array<double, 7> joint_positions;
  bool motion_finished = (time_ >= t_wave_end_ + backing_time_);

  if (time_ < t_wave_end_) {
    joint_positions =
        CosineWavePosition(time_, initial_joint_position_, A_, omega_);
  } else {
    joint_positions =
        PolynomialPosition(time_, backing_trajectory_coefficients_);
  }

  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}