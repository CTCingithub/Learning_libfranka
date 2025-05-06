#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "Controllers.h"
#include "CosineWaveGenerator.h"
#include "DefaultBehavior.h"
#include "EnableArrayPrint.h"
#include "ReadConfig.h"

struct state_logger {
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> tau;
};

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
  std::array<double, 7> Kp =
      controller_config["Kp"].as<std::array<double, 7>>();
  std::array<double, 7> Kd =
      controller_config["Kd"].as<std::array<double, 7>>();
  std::array<double, 7> amplitudes =
      controller_config["Amplitude"].as<std::array<double, 7>>();
  std::array<double, 7> omegas =
      controller_config["Omega"].as<std::array<double, 7>>();
  bool coriolis_compensation =
      controller_config["Coriolis_Compensation"].as<bool>();
  std::cout << "Coriolis Compensation: " << coriolis_compensation << std::endl;
  bool sampling_info = controller_config["Sampling_Info"].as<bool>();
  std::cout << "Sampling Info: " << sampling_info << std::endl;

  const int ts = controller_config["Sampling_Time"].as<int>();
  const double t_switch = controller_config["Switch_Time"].as<double>();
  const double t_back = controller_config["Backing_Time"].as<double>();
  std::cout << "Sampling time: " << ts << " ms" << std::endl;

  // Parameters
  const size_t filter_size{1};

  Controllers::PDController pd_controller(filter_size, Kp, Kd,
                                          coriolis_compensation);

  try {
    franka::Robot Myrobot(robot_ip);
    franka::Model Mymodel(Myrobot.loadModel());
    Myrobot.automaticErrorRecovery();

    Reach_Desired_Joint_Pose(Myrobot);
    Reach_Desired_Joint_Pose(Myrobot, q_desired);
    set_default_collision(Myrobot);

    double t = 0.0;
    int timer = 0;
    franka::RobotState robot_state_sampled = Myrobot.readOnce();

    auto callback_control_torque =
        [&](const franka::RobotState &robot_state,
            franka::Duration period) -> franka::Torques {
      // Manual switch sampling frequency
      timer += period.toMSec();
      if (timer % ts == 0) {
        robot_state_sampled = robot_state;
        if (sampling_info) {
          std::cout << "Sample at " << timer << "ms" << std::endl;
        }
      }
      return pd_controller.step(robot_state_sampled, Mymodel);
    };

    CosineWaveMotionGenerator motion_generator(robot_state_sampled.q,
                                               amplitudes, omegas, t_switch);

    bool motion_finished = false;
    auto active_control = Myrobot.startJointPositionControl(
        research_interface::robot::Move::ControllerMode::kExternalController);

    while (!motion_finished) {
      auto read_once_return = active_control->readOnce();
      auto robot_state = read_once_return.first;
      auto duration = read_once_return.second;
      auto joint_positions = motion_generator(robot_state, duration);
      auto torques = callback_control_torque(robot_state, duration);
      motion_finished = joint_positions.motion_finished;
      active_control->writeOnce(joint_positions, torques);
    }
    // Myrobot.control(motion_generator);
    Myrobot.automaticErrorRecovery();
    Reach_Desired_Joint_Pose(Myrobot);
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
