#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "DefaultBehavior.h"
#include "EnableArrayPrint.h"
#include "ReadConfig.h"

namespace {
Eigen::MatrixXd getDefaultStiffness() {
  Eigen::MatrixXd stiffness(6, 6);
  stiffness.setZero();
  const double translational_stiffness = 150.0;
  const double rotational_stiffness = 10.0;
  stiffness.topLeftCorner(3, 3) =
      translational_stiffness * Eigen::Matrix3d::Identity();
  stiffness.bottomRightCorner(3, 3) =
      rotational_stiffness * Eigen::Matrix3d::Identity();
  return stiffness;
}

Eigen::MatrixXd getDefaultDamping() {
  Eigen::MatrixXd damping(6, 6);
  damping.setZero();
  const double translational_stiffness = 150.0;
  const double rotational_stiffness = 10.0;
  damping.topLeftCorner(3, 3) =
      2.0 * sqrt(translational_stiffness) * Eigen::Matrix3d::Identity();
  damping.bottomRightCorner(3, 3) =
      2.0 * sqrt(rotational_stiffness) * Eigen::Matrix3d::Identity();
  return damping;
}
} // namespace

std::array<double, 7> cartesian_impedance_feedback_func(
    const franka::RobotState &robot_state, const franka::Model &model,
    const Eigen::Vector3d &EE_position_desired,
    const Eigen::Quaterniond &EE_orientation_desired,
    const Eigen::MatrixXd &stiffness = getDefaultStiffness(),
    const Eigen::MatrixXd &damping = getDefaultDamping()) {
  // Get state variables
  std::array<double, 7> coriolis_array = model.coriolis(robot_state);
  std::array<double, 42> jacobian_array =
      model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

  // Convert to Eigen
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d EE_position(transform.translation());
  Eigen::Quaterniond EE_orientation(transform.rotation());

  // Compute error to desired equilibrium pose
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) = EE_position - EE_position_desired;

  // Orientation error
  if (EE_orientation_desired.coeffs().dot(EE_orientation.coeffs()) < 0.0) {
    EE_orientation.coeffs() = -EE_orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(EE_orientation.inverse() *
                                      EE_orientation_desired);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(),
      error_quaternion.z();
  error.tail(3) = -transform.rotation() * error.tail(3);

  // Compute control
  Eigen::VectorXd tau_task(7), tau_d(7);
  tau_task =
      jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
  tau_d = tau_task + coriolis;

  std::array<double, 7> tau_d_array;
  Eigen::VectorXd::Map(tau_d_array.data(), 7) = tau_d;

  return tau_d_array;
}

int main(int argc, char **argv) {
  YAML::Node config = GetConfig();
  const std::string robot_ip = config["Robot_ip"].as<std::string>();
  std::array<double, 7> q_zero = config["Zero_Pos"].as<std::array<double, 7>>();
  std::cout << "IP Address of Franka Research 3 Robot: " << robot_ip
            << std::endl;
  std::cout << "Zero Position: " << q_zero << std::endl;

  try {
    franka::Robot Myrobot(robot_ip);
    franka::RobotState current_state = Myrobot.readOnce();
    franka::Model Mymodel(Myrobot.loadModel());

    set_default_collision(Myrobot);
    std::cout << "Current Joint Position: " << current_state.q << std::endl;

    Eigen::Affine3d initial_transform_mat(
        Eigen::Matrix4d::Map(current_state.O_T_EE.data()));
    Eigen::Vector3d initial_position = initial_transform_mat.translation();
    Eigen::Quaterniond initial_orientation(initial_transform_mat.rotation());

    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        controller_callback =
            [&](const franka::RobotState &robot_state,
                franka::Duration /*duration*/) -> franka::Torques {
      return cartesian_impedance_feedback_func(
          robot_state, Mymodel, initial_position, initial_orientation);
    };
    Myrobot.control(controller_callback);
  } catch (franka::Exception const &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
