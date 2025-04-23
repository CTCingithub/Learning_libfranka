#pragma once

#include "Eigen/Dense"
#include "franka/model.h"
#include "franka/robot.h"

namespace Controllers {
class PDController {
public:
  PDController(size_t dq_filter_size, const std::array<double, 7> &K_P,
               const std::array<double, 7> &K_D);

  franka::Torques step(const franka::RobotState &state,
                       const franka::Model &model);
  void updateDQFilter(const franka::RobotState &state);
  double getDQFiltered(size_t index) const;

private:
  size_t dq_current_filter_position_;
  size_t dq_filter_size_;

  const std::array<double, 7> K_P_;
  const std::array<double, 7> K_D_;

  std::array<double, 7> dq_d_;
  std::unique_ptr<double[]> dq_buffer_;
};
} // namespace Controllers