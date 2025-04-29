#include <array>
#include <cmath>

#include "Eigen/Dense"

#include "PolynomialMotionGenerator.h"

/*
Cubic Polynomial Motion Generator
q(t) = a0 + a1*t + a2*t^2 + a3*t^3,
\dot{q}(t) = a1 + 2*a2*t + 3*a3*t^2,
*/

std::array<double, 4>
CubicPolynomialMotionGenerator::Cubic_Polynomial_Coefficients(
    const double &duration, const double &position_start,
    const double &velocity_start, const double &position_end,
    const double &velocity_end) {
  std::array<double, 4> coefficients = {position_start, velocity_start, 0, 0};
  Eigen::Matrix<double, 2, 2> A;
  A << pow(duration, 2), pow(duration, 3), // end time position
      2 * duration, 3 * pow(duration, 2);  // end time velocity
  Eigen::Matrix<double, 2, 1> b;
  b << position_end - position_start -
           duration * velocity_start, // end time position
      velocity_end - velocity_start;  // end time velocity
  auto solution = A.colPivHouseholderQr().solve(b);
  for (size_t i = 2; i < 4; i++) {
    coefficients[i] = solution(i - 2);
  }
  return coefficients;
}

std::array<double, 7> CubicPolynomialMotionGenerator::Cubic_Polynomial_Position(
    const double &time,
    const std::array<std::array<double, 4>, 7> &trajectory_coefficients) {
  std::array<double, 7> position;
  for (size_t i = 0; i < 7; i++) {
    std::array<double, 4> coefficients = trajectory_coefficients[i];
    position[i] = coefficients[0] + coefficients[1] * time +
                  coefficients[2] * pow(time, 2) +
                  coefficients[3] * pow(time, 3);
  }
  return position;
}

std::array<double, 7> CubicPolynomialMotionGenerator::Cubic_Polynomial_Velocity(
    const double &time,
    const std::array<std::array<double, 4>, 7> &trajectory_coefficients) {
  std::array<double, 7> velocity;
  for (size_t i = 0; i < 7; i++) {
    std::array<double, 4> coefficients = trajectory_coefficients[i];
    velocity[i] = coefficients[1] + 2 * coefficients[2] * time +
                  3 * coefficients[3] * pow(time, 2);
  }
  return velocity;
}

std::array<double, 7>
CubicPolynomialMotionGenerator::Cubic_Polynomial_Acceleration(
    const double &time,
    const std::array<std::array<double, 4>, 7> &trajectory_coefficients) {
  std::array<double, 7> acceleration;
  for (size_t i = 0; i < 7; i++) {
    std::array<double, 4> coefficients = trajectory_coefficients[i];
    acceleration[i] = 2 * coefficients[2] + 6 * coefficients[3] * time;
  }
  return acceleration;
}

CubicPolynomialMotionGenerator::CubicPolynomialMotionGenerator(
    const double &duration, const std::array<double, 7> &initial_joint_position,
    const std::array<double, 7> &initial_joint_velocity,
    const std::array<double, 7> &final_joint_position,
    const std::array<double, 7> &final_joint_velocity)
    : duration_(duration), initial_joint_position_(initial_joint_position),
      initial_joint_velocity_(initial_joint_velocity),
      final_joint_position_(final_joint_position),
      final_joint_velocity_(final_joint_velocity) {
  duration_ = duration;
  initial_joint_position_ = initial_joint_position;
  initial_joint_velocity_ = initial_joint_velocity;
  final_joint_position_ = final_joint_position;
  final_joint_velocity_ = final_joint_velocity;
  time_ = 0.0;

  for (size_t i = 0; i < 7; i++) {
    trajectory_coefficients_[i] = Cubic_Polynomial_Coefficients(
        duration_, initial_joint_position_[i], initial_joint_velocity_[i],
        final_joint_position_[i], final_joint_velocity_[i]);
  }

  q_end_ = Cubic_Polynomial_Position(duration_, trajectory_coefficients_);
  dq_end_ = Cubic_Polynomial_Velocity(duration_, trajectory_coefficients_);
  ddq_end_ = Cubic_Polynomial_Acceleration(duration_, trajectory_coefficients_);
}

franka::JointPositions CubicPolynomialMotionGenerator::operator()(
    const franka::RobotState &robot_state, franka::Duration period) {
  time_ += period.toSec();
  bool motion_finished = (time_ >= duration_);

  std::array<double, 7> joint_positions =
      Cubic_Polynomial_Position(time_, trajectory_coefficients_);

  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}

std::array<double, 7>
CubicPolynomialMotionGenerator::trajectory_position(const double &time) {
  return Cubic_Polynomial_Position(time, trajectory_coefficients_);
}

std::array<double, 7>
CubicPolynomialMotionGenerator::trajectory_velocity(const double &time) {
  return Cubic_Polynomial_Velocity(time, trajectory_coefficients_);
}

std::array<double, 7>
CubicPolynomialMotionGenerator::trajectory_acceleration(const double &time) {
  return Cubic_Polynomial_Acceleration(time, trajectory_coefficients_);
}

std::array<std::array<double, 7>, 3>
CubicPolynomialMotionGenerator::kinematics_end() {
  std::array<std::array<double, 7>, 3> kinematics_end;
  kinematics_end[0] = q_end_;
  kinematics_end[1] = dq_end_;
  kinematics_end[2] = ddq_end_;
  return kinematics_end;
}

/*
Quintic Polynomial Motion Generator
q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
\dot{q}(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
\ddot{q}(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
*/

std::array<double, 6>
Quintic_Polynomial_Motion_Generator::Quintic_Polynomial_Coefficients(
    const double &duration, const double &position_start,
    const double &velocity_start, const double &acceleration_start,
    const double &position_end, const double &velocity_end,
    const double &acceleration_end) {
  std::array<double, 6> coefficients = {
      position_start, velocity_start, acceleration_start / 2, 0, 0, 0};
  Eigen::Matrix<double, 3, 3> A;
  A << pow(duration, 3), pow(duration, 4),
      pow(duration, 5), // end time position
      3 * pow(duration, 2), 4 * pow(duration, 3),
      5 * pow(duration, 4), // end time velocity
      6 * duration, 12 * pow(duration, 2),
      20 * pow(duration, 3); // end time acceleration
  Eigen::Matrix<double, 3, 1> b;
  b << position_end - coefficients[0] - coefficients[1] * duration -
           coefficients[2] * pow(duration, 2), // end time position
      velocity_end - coefficients[1] -
          2 * coefficients[2] * duration,     // end time velocity
      acceleration_end - 2 * coefficients[2]; // end time acceleration
  auto solution = A.colPivHouseholderQr().solve(b);
  for (size_t i = 3; i < 6; i++)
    coefficients[i] = solution(i - 3);
  return coefficients;
}

std::array<double, 7>
Quintic_Polynomial_Motion_Generator::Quintic_Polynomial_Position(
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

std::array<double, 7>
Quintic_Polynomial_Motion_Generator::Quintic_Polynomial_Velocity(
    const double &time,
    const std::array<std::array<double, 6>, 7> &trajectory_coefficients) {
  std::array<double, 7> velocity;
  for (size_t i = 0; i < 7; i++) {
    std::array<double, 6> coefficients = trajectory_coefficients[i];
    velocity[i] = coefficients[1] + 2 * coefficients[2] * time +
                  3 * coefficients[3] * pow(time, 2) +
                  4 * coefficients[4] * pow(time, 3) +
                  5 * coefficients[5] * pow(time, 4);
  }
  return velocity;
}

std::array<double, 7>
Quintic_Polynomial_Motion_Generator::Quintic_Polynomial_Acceleration(
    const double &time,
    const std::array<std::array<double, 6>, 7> &trajectory_coefficients) {
  std::array<double, 7> acceleration;
  for (size_t i = 0; i < 7; i++) {
    std::array<double, 6> coefficients = trajectory_coefficients[i];
    acceleration[i] = 2 * coefficients[2] + 6 * coefficients[3] * time +
                      12 * coefficients[4] * pow(time, 2) +
                      20 * coefficients[5] * pow(time, 3);
  }
  return acceleration;
}

Quintic_Polynomial_Motion_Generator::Quintic_Polynomial_Motion_Generator(
    const double &duration, const std::array<double, 7> &initial_joint_position,
    const std::array<double, 7> &initial_joint_velocity,
    const std::array<double, 7> &initial_joint_acceleration,
    const std::array<double, 7> &final_joint_position,
    const std::array<double, 7> &final_joint_velocity,
    const std::array<double, 7> &final_joint_acceleration)
    : duration_(duration), initial_joint_position_(initial_joint_position),
      initial_joint_velocity_(initial_joint_velocity),
      initial_joint_acceleration_(initial_joint_acceleration),
      final_joint_position_(final_joint_position),
      final_joint_velocity_(final_joint_velocity),
      final_joint_acceleration_(final_joint_acceleration) {
  duration_ = duration;
  initial_joint_position_ = initial_joint_position;
  initial_joint_velocity_ = initial_joint_velocity;
  initial_joint_acceleration_ = initial_joint_acceleration;
  final_joint_position_ = final_joint_position;
  final_joint_velocity_ = final_joint_velocity;
  final_joint_acceleration_ = final_joint_acceleration;
  time_ = 0.0;

  for (size_t i = 0; i < 7; ++i) {
    trajectory_coefficients_[i] = Quintic_Polynomial_Coefficients(
        duration_, initial_joint_position_[i], initial_joint_velocity_[i],
        initial_joint_acceleration_[i], final_joint_position_[i],
        final_joint_velocity_[i], final_joint_acceleration_[i]);
  }

  q_end_ = Quintic_Polynomial_Acceleration(duration_, trajectory_coefficients_);
  dq_end_ = Quintic_Polynomial_Velocity(duration_, trajectory_coefficients_);
  ddq_end_ =
      Quintic_Polynomial_Acceleration(duration_, trajectory_coefficients_);
}

franka::JointPositions Quintic_Polynomial_Motion_Generator::operator()(
    const franka::RobotState &robot_state, franka::Duration period) {
  time_ += period.toSec();
  bool motion_finished = (time_ >= duration_);

  std::array<double, 7> joint_positions =
      Quintic_Polynomial_Position(time_, trajectory_coefficients_);

  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}

std::array<double, 7>
Quintic_Polynomial_Motion_Generator::trajectory_position(const double &time) {
  return Quintic_Polynomial_Position(time, trajectory_coefficients_);
}

std::array<double, 7>
Quintic_Polynomial_Motion_Generator::trajectory_velocity(const double &time) {
  return Quintic_Polynomial_Velocity(time, trajectory_coefficients_);
}

std::array<double, 7>
Quintic_Polynomial_Motion_Generator::trajectory_acceleration(
    const double &time) {
  return Quintic_Polynomial_Acceleration(time, trajectory_coefficients_);
}

std::array<std::array<double, 7>, 3>
Quintic_Polynomial_Motion_Generator::kinematics_end() {
  std::array<std::array<double, 7>, 3> kinematics_end;
  kinematics_end[0] = q_end_;
  kinematics_end[1] = dq_end_;
  kinematics_end[2] = ddq_end_;
  return kinematics_end;
}