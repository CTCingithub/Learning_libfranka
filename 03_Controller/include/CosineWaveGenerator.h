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
  double t_wave_end_;
  double time_;
  double backing_time_;
};