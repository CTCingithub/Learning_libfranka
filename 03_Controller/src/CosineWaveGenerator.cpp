#include "CosineWaveGenerator.h"
#include <cmath>

CosineMotionGenerator::CosineMotionGenerator(
    const std::array<double, 7> &initial_joint_position,
    const std::array<double, 7> &A, const std::array<double, 7> &omega,
    double t_end)
    : initial_joint_position_(initial_joint_position), A_(A), omega_(omega),
      t_end_(t_end) {}

franka::JointPositions
CosineMotionGenerator::operator()(const franka::RobotState &robot_state,
                                  franka::Duration period) {
  time_ += period.toSec();

  std::array<double, 7> joint_positions;
  bool motion_finished = (time_ >= t_end_);

  for (size_t i = 0; i < 7; ++i) {
    if (motion_finished) {
      joint_positions[i] = initial_joint_position_[i];
    } else {
      double t = time_;
      // ���ζ���ʽ���ں�����ȷ��t_endʱƽ������
      double window =
          1.0 - 3.0 * std::pow(t / t_end_, 2) + 2.0 * std::pow(t / t_end_, 3);
      // ���Ҳ��켣
      double cos_term = 1.0 - std::cos(omega_[i] * t);
      joint_positions[i] =
          initial_joint_position_[i] + A_[i] * cos_term * window;
    }
  }

  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}