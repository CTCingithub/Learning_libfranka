#include "Controllers.h"

#include "Eigen/Dense"
#include "franka/model.h"
#include "franka/robot.h"

namespace Controllers {
PDController::PDController(size_t dq_filter_size,
                           const std::array<double, 7> &K_P,
                           const std::array<double, 7> &K_D,
                           const bool &coriolis_compensation,
                           const bool &gravity_compensation)
    : dq_current_filter_position_(0), dq_filter_size_(dq_filter_size),
      K_P_(K_P), K_D_(K_D), coriolis_compensation_(coriolis_compensation),
      gravity_compensation_(gravity_compensation) {
  std::fill(dq_d_.begin(), dq_d_.end(), 0);
  dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
  std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
  gravity_compensation_ = gravity_compensation;
}

franka::Torques PDController::step(const franka::RobotState &state,
                                   const franka::Model &model) {
  updateDQFilter(state);

  std::array<double, 7> tau_J_d;
  std::array<double, 7> coriolis_array = model.coriolis(state);
  std::array<double, 7> gravity_array = model.gravity(state);
  for (size_t i = 0; i < 7; i++) {
    tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) +
                 K_D_[i] * (dq_d_[i] - getDQFiltered(i));
    if (coriolis_compensation_) {
      tau_J_d[i] += coriolis_array[i];
    }
    if (gravity_compensation_) {
      tau_J_d[i] += gravity_array[i];
    }
  }
  return tau_J_d;
}

void PDController::updateDQFilter(const franka::RobotState &state) {
  for (size_t i = 0; i < 7; i++) {
    dq_buffer_.get()[dq_current_filter_position_ * 7 + i] = state.dq[i];
  }
  dq_current_filter_position_ =
      (dq_current_filter_position_ + 1) % dq_filter_size_;
}

double PDController::getDQFiltered(size_t index) const {
  double value = 0;
  for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
    value += dq_buffer_.get()[i];
  }
  return value / dq_filter_size_;
}
} // namespace Controllers