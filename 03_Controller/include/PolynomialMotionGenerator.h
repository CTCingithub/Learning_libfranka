#pragma once

#include <array>
#include <cmath>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

class CubicPolynomialMotionGenerator {
public:
  CubicPolynomialMotionGenerator(
      const double &duration,
      const std::array<double, 7> &initial_joint_position,
      const std::array<double, 7> &initial_joint_velocity,
      const std::array<double, 7> &final_joint_position,
      const std::array<double, 7> &final_joint_velocity = {0, 0, 0, 0, 0, 0,
                                                           0});

  franka::JointPositions operator()(const franka::RobotState &robot_state,
                                    franka::Duration period);

  std::array<double, 7> trajectory_position(const double &time);
  std::array<double, 7> trajectory_velocity(const double &time);
  std::array<double, 7> trajectory_acceleration(const double &time);
  std::array<std::array<double, 7>, 3> kinematics_end();

private:
  double duration_;
  std::array<double, 7> initial_joint_position_;
  std::array<double, 7> initial_joint_velocity_;
  std::array<double, 7> final_joint_position_;
  std::array<double, 7> final_joint_velocity_;

  double time_;
  std::array<std::array<double, 4>, 7> trajectory_coefficients_;

  std::array<double, 7> q_end_;
  std::array<double, 7> dq_end_;
  std::array<double, 7> ddq_end_;

protected:
  std::array<double, 4> Cubic_Polynomial_Coefficients(
      const double &duration, const double &position_start,
      const double &velocity_start, const double &position_end,
      const double &velocity_end);
  std::array<double, 7> Cubic_Polynomial_Position(
      const double &time,
      const std::array<std::array<double, 4>, 7> &trajectory_coefficients);
  std::array<double, 7> Cubic_Polynomial_Velocity(
      const double &time,
      const std::array<std::array<double, 4>, 7> &trajectory_coefficients);
  std::array<double, 7> Cubic_Polynomial_Acceleration(
      const double &time,
      const std::array<std::array<double, 4>, 7> &trajectory_coefficients);
};

class Quintic_Polynomial_Motion_Generator {
public:
  Quintic_Polynomial_Motion_Generator(
      const double &duration,
      const std::array<double, 7> &initial_joint_position,
      const std::array<double, 7> &initial_joint_velocity,
      const std::array<double, 7> &initial_joint_acceleration,
      const std::array<double, 7> &final_joint_position,
      const std::array<double, 7> &final_joint_velocity = {0, 0, 0, 0, 0, 0, 0},
      const std::array<double, 7> &final_joint_acceleration = {0, 0, 0, 0, 0, 0,
                                                               0});

  franka::JointPositions operator()(const franka::RobotState &robot_state,
                                    franka::Duration period);

  std::array<double, 7> trajectory_position(const double &time);
  std::array<double, 7> trajectory_velocity(const double &time);
  std::array<double, 7> trajectory_acceleration(const double &time);
  std::array<std::array<double, 7>, 3> kinematics_end();

private:
  double duration_;
  std::array<double, 7> initial_joint_position_;
  std::array<double, 7> initial_joint_velocity_;
  std::array<double, 7> initial_joint_acceleration_;
  std::array<double, 7> final_joint_position_;
  std::array<double, 7> final_joint_velocity_;
  std::array<double, 7> final_joint_acceleration_;

  std::array<double, 7> q_end_;
  std::array<double, 7> dq_end_;
  std::array<double, 7> ddq_end_;

  double time_;
  std::array<std::array<double, 6>, 7> trajectory_coefficients_;

protected:
  std::array<double, 6> Quintic_Polynomial_Coefficients(
      const double &duration, const double &position_start,
      const double &velocity_start, const double &acceleration_start,
      const double &position_end, const double &velocity_end,
      const double &acceleration_end);
  std::array<double, 7> Quintic_Polynomial_Position(
      const double &time,
      const std::array<std::array<double, 6>, 7> &trajectory_coefficients);
  std::array<double, 7> Quintic_Polynomial_Velocity(
      const double &time,
      const std::array<std::array<double, 6>, 7> &trajectory_coefficients);
  std::array<double, 7> Quintic_Polynomial_Acceleration(
      const double &time,
      const std::array<std::array<double, 6>, 7> &trajectory_coefficients);
};