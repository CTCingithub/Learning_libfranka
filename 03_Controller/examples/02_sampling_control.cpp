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

struct sampled_states_struct {
  std::array<double, 7> q_sampled;
  std::array<double, 7> dq_sampled;
  franka::RobotState state_sampled;
  std::array<double, 7> torques;
};

std::array<double, 7> PD_Feedback_Func(
    const franka::Model &model, const franka::RobotState &sampled_state,
    const std::array<double, 7> &Kp, const std::array<double, 7> &Kd,
    const std::array<double, 7> &q_sampled,
    const std::array<double, 7> &dq_sampled) {
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_desired(
      sampled_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix_desired(
      model.mass(sampled_state).data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_compensation(
      model.gravity(sampled_state).data());
  Eigen::Matrix<double, 7, 1> e_sampled;
  for (size_t i = 0; i < 7; ++i) {
    e_sampled[i] = q_sampled[i] - q_desired[i];
  }
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> de_sampled(dq_sampled.data());
  Eigen::DiagonalMatrix<double, 7> Kp_matrix(
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>(Kp.data()));
  Eigen::DiagonalMatrix<double, 7> Kd_matrix(
      Eigen::Map<const Eigen::Matrix<double, 7, 1>>(Kd.data()));
  Eigen::Matrix<double, 7, 1> tau =
      gravity_compensation - Kp_matrix * e_sampled - Kd_matrix * de_sampled;

  // std::array<double, 7> tau_array;
  // Eigen::MatrixXd::Map(tau_array.data(), 7) = tau;
  std::array<double, 7> tau_array;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_map(tau_array.data());
  tau_map = tau;

  return tau_array;
}

int main(int argc, char **argv) {
  // Read config file
  std::string robot_config_file = FindConfigFile("FR3.yaml");
  std::string controller_config_file = FindConfigFile("PD_Control.yaml");
  YAML::Node robot_config = GetConfig(robot_config_file);
  YAML::Node controller_config = GetConfig(controller_config_file);
  const std::string robot_ip = robot_config["Robot_ip"].as<std::string>();
  std::cout << "IP Address of Franka Research 3 Robot: " << robot_ip
            << std::endl;
  std::array<double, 7> q_desired =
      controller_config["Desired_Pos"].as<std::array<double, 7>>();
  std::array<double, 7> e_initial =
      controller_config["Initial_Error"].as<std::array<double, 7>>();
  std::array<double, 7> Kp =
      controller_config["Kp"].as<std::array<double, 7>>();
  std::array<double, 7> Kd =
      controller_config["Kd"].as<std::array<double, 7>>();
  std::array<double, 7> q_initial;

  // Generate initial position
  for (size_t i = 0; i < q_desired.size(); i++) {
    q_initial[i] = q_desired[i] + e_initial[i];
  }
  const int ts = controller_config["Sampling_Time"].as<int>();
  std::cout << "Sampling time: " << ts << " ms" << std::endl;

  try {
    franka::Robot Myrobot(robot_ip);
    Reach_Desired_Joint_Pose(Myrobot, q_desired);
    franka::RobotState sampled_state = Myrobot.readOnce();
    Reach_Desired_Joint_Pose(Myrobot);

    std::cout << "Moving to initial position" << std::endl;
    Reach_Desired_Joint_Pose(Myrobot, q_initial);
    franka::RobotState initial_state = Myrobot.readOnce();
    franka::Model Mymodel(Myrobot.loadModel());
    std::cout << "Reached initial position" << std::endl;

    int time = 0;
    sampled_states_struct SampledStruct;
    SampledStruct.q_sampled = initial_state.q;
    SampledStruct.dq_sampled = initial_state.dq;
    SampledStruct.state_sampled = initial_state;
    SampledStruct.torques = initial_state.tau_J;

    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        controller_callback = [&](const franka::RobotState &robot_state,
                                  franka::Duration period) -> franka::Torques {
      time += period.toMSec();
      if (time % ts == 0) {
        SampledStruct.q_sampled = robot_state.q;
        SampledStruct.dq_sampled = robot_state.dq;
        SampledStruct.state_sampled = robot_state;
        SampledStruct.torques = robot_state.tau_J;
      }
      return PD_Feedback_Func(Mymodel, SampledStruct.state_sampled, Kp, Kd,
                              SampledStruct.q_sampled,
                              SampledStruct.dq_sampled);
    };
    Myrobot.control(controller_callback);
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
  }

  return 0;
}