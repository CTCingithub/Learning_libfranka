#pragma once

#include "Eigen/Dense"
#include "franka/model.h"
#include "franka/robot.h"
#include <tuple>

namespace Controllers {
class PDController {
public:
  PDController(const std::array<double, 7> &K_P,
               const std::array<double, 7> &K_D,
               const bool &coriolis_compensation = false,
               size_t dq_filter_size = 1);

  franka::Torques step(const franka::RobotState &state,
                       const franka::Model &model);
  franka::Torques step(const std::array<double, 7> &q,
                       const std::array<double, 7> &dq,
                       const std::array<double, 7> &q_d,
                       const std::array<double, 7> &dq_d,
                       const franka::RobotState &state,
                       const franka::Model &model);
  void updateDQFilter(const franka::RobotState &state);
  double getDQFiltered(size_t index) const;

private:
  size_t dq_current_filter_position_;
  size_t dq_filter_size_;

  const std::array<double, 7> K_P_;
  const std::array<double, 7> K_D_;
  bool coriolis_compensation_;

  std::unique_ptr<double[]> dq_buffer_;
};

class ComputedTorqueController {
public:
  ComputedTorqueController(const std::array<double, 7> &K_P,
                           const std::array<double, 7> &K_D);

  franka::Torques step(const franka::RobotState &state,
                       const franka::Model &model);
  void updateFilter(const franka::RobotState &state);
  std::tuple<std::array<double, 7>, std::array<double, 7>>
  getFiltered(const franka::RobotState &state) const;

private:
  Eigen::Matrix<double, 7, 7> K_P_;
  Eigen::Matrix<double, 7, 7> K_D_;

  std::array<double, 7> tau_J_d_;
};
} // namespace Controllers