#include "Controllers.h"

#include "Eigen/Dense"
#include "franka/model.h"
#include "franka/robot.h"

namespace Controllers {
PDController::PDController(const std::array<double, 7> &K_P,
                           const std::array<double, 7> &K_D,
                           const bool &coriolis_compensation,
                           size_t dq_filter_size)
    : dq_current_filter_position_(0), dq_filter_size_(dq_filter_size),
      K_P_(K_P), K_D_(K_D), coriolis_compensation_(coriolis_compensation) {
  dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
  std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
}

franka::Torques PDController::step(const franka::RobotState &state,
                                   const franka::Model &model) {
  updateDQFilter(state);

  std::array<double, 7> tau_J_d;
  std::array<double, 7> coriolis_array = model.coriolis(state);
  for (size_t i = 0; i < 7; i++) {
    tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) +
                 K_D_[i] * (state.dq_d[i] - getDQFiltered(i));
    if (coriolis_compensation_) {
      tau_J_d[i] += coriolis_array[i];
    }
  }
  return tau_J_d;
}

franka::Torques PDController::step(const std::array<double, 7> &q,
                                   const std::array<double, 7> &dq,
                                   const std::array<double, 7> &q_d,
                                   const std::array<double, 7> &dq_d,
                                   const franka::RobotState &state,
                                   const franka::Model &model) {
  std::array<double, 7> tau_J_d;
  std::array<double, 7> coriolis_array = model.coriolis(state);
  for (size_t i = 0; i < 7; i++) {
    tau_J_d[i] =
        K_P_[i] * (q_d[i] - q[i]) + K_D_[i] * (dq_d[i] - dq[i]);
    if (coriolis_compensation_) {
      tau_J_d[i] += coriolis_array[i];
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

ComputedTorqueController::ComputedTorqueController(
    const std::array<double, 7> &K_P, const std::array<double, 7> &K_D) {
  Eigen::Matrix<double, 7, 1> Kp_elements(K_P.data());
  Eigen::Matrix<double, 7, 1> Kd_elements(K_D.data());
  K_P_ = Kp_elements.asDiagonal();
  K_D_ = Kd_elements.asDiagonal();
}

franka::Torques ComputedTorqueController::step(const franka::RobotState &state,
                                               const franka::Model &model) {
  // Update filter
  updateFilter(state);

  // Get dynamical terms
  auto MassMatrix =
      Eigen::Map<Eigen::Matrix<double, 7, 7>>(model.mass(state).data());
  auto coriolis =
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(model.coriolis(state).data());
  // gravity is compensated in HAL or controller

  // Get desired joint positions and velocities
  auto q_d_array = state.q_d;
  auto dq_d_array = state.dq_d;
  auto ddq_d_array = state.ddq_d;
  auto q_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_d_array.data());
  auto dq_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(dq_d_array.data());
  auto ddq_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(ddq_d_array.data());

  // Get filtered joint positions and velocities
  auto state_filtered_array = getFiltered(state);
  auto q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(
      std::get<0>(state_filtered_array).data());
  auto dq = Eigen::Map<Eigen::Matrix<double, 7, 1>>(
      std::get<1>(state_filtered_array).data());

  // Get position error and velocity error
  auto q_error = q_d - q;
  auto dq_error = dq_d - dq;

  // Compute torque
  auto tau_J_d =
      MassMatrix * ddq_d + coriolis + K_P_ * q_error + K_D_ * dq_error;
  Eigen::Map<Eigen::VectorXd>(tau_J_d_.data(), 7) = tau_J_d;
  return tau_J_d_;
}

void ComputedTorqueController::updateFilter(const franka::RobotState &state) {}

std::tuple<std::array<double, 7>, std::array<double, 7>>
ComputedTorqueController::getFiltered(const franka::RobotState &state) const {
  std::array<double, 7> q_filtered, dq_filtered;
  std::array<double, 7> q_sensor = state.q;
  std::array<double, 7> dq_sensor = state.dq;
  for (size_t i = 0; i < 7; i++) {
    q_filtered[i] = q_sensor[i];
    dq_filtered[i + 7] = dq_sensor[i];
  }
  return {q_filtered, dq_filtered};
}
} // namespace Controllers