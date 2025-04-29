#pragma once

#include <array>
#include <cmath>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

class CosineWaveMotionGenerator {
public:
  CosineWaveMotionGenerator(const std::array<double, 7> &initial_joint_position,
                            const std::array<double, 7> &A,
                            const std::array<double, 7> &omega,
                            const double &duration);

  franka::JointPositions operator()(const franka::RobotState &robot_state,
                                    franka::Duration period);

  std::array<double, 7> trajectory_position(const double &time);
  std::array<double, 7> trajectory_velocity(const double &time);
  std::array<double, 7> trajectory_acceleration(const double &time);
  std::array<std::array<double, 7>, 3> kinematics_end();

private:
  double duration_;
  std::array<double, 7> initial_joint_position_;
  std::array<double, 7> A_;
  std::array<double, 7> omega_;

  double time_;

  std::array<double, 7> q_end_;
  std::array<double, 7> dq_end_;
  std::array<double, 7> ddq_end_;

protected:
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
};