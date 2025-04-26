#pragma once

#include <array>
#include <cmath>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

class CosineMotionGenerator {
public:
  CosineMotionGenerator(const std::array<double, 7> &initial_joint_position,
                        const std::array<double, 7> &A,
                        const std::array<double, 7> &omega,
                        const double &t_wave_end, const double &backing_time);

  franka::JointPositions operator()(const franka::RobotState &robot_state,
                                    franka::Duration period);

private:
  std::array<double, 7> initial_joint_position_;
  std::array<double, 7> A_;
  std::array<double, 7> omega_;

  double time_;
  double t_wave_end_;
  double backing_time_;

  std::array<double, 7> q_wave_end_;
  std::array<double, 7> dq_wave_end_;
  std::array<double, 7> ddq_wave_end_;
  std::array<std::array<double, 6>, 7> backing_trajectory_coefficients_;
  std::array<double, 7>
  CosineWavePosition(const double time,
                     const std::array<double, 7> &Initial_Positions,
                     const std::array<double, 7> &Amplitudes,
                     const std::array<double, 7> &Omegas);
  std::array<double, 7>
  CosineWaveVelocity(const double time, const std::array<double, 7> &Amplitudes,
                     const std::array<double, 7> &Omegas);
  std::array<double, 7>
  CosineWaveAcceleration(const double time,
                         const std::array<double, 7> &Amplitudes,
                         const std::array<double, 7> &Omegas);
  std::array<double, 6> Polynomial_Coefficients(
      const double &time_start, const double &time_end,
      const double &position_start, const double &velocity_start,
      const double &acceleration_start, const double &position_end,
      const double &velocity_end, const double &acceleration_end);
  std::array<double, 7> PolynomialPosition(
      const double &time,
      const std::array<std::array<double, 6>, 7> &trajectory_coefficients);
};