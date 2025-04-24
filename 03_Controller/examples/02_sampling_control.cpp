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
#include "DefaultBehavior.h"
#include "EnableArrayPrint.h"
#include "ReadConfig.h"

struct state_logger {
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> tau;
};

namespace {
std::array<double, 7>
CosineWaveTrajectory(const double time, const std::array<double, 7> &Amplitudes,
                     const std::array<double, 7> &Omegas) {
  /*
  Cosine-wave trajectory, q_i(t) = q_i(0) + A_i * (1 - cos(omega_i * t)
  Velocity follows \dot{q}_i(t) = -A_i * omega_i * sin(omega_i * t)
  Use `JointVelocities` mode to eliminate initial position inputs
  */
  constexpr double dt = 0.001;
  std::array<double, 7> velocity; // [rad/s]
  for (int i = 0; i < 7; i++) {
    velocity[i] = -Amplitudes[i] * Omegas[i] * sin(Omegas[i] * time);
  }
  return velocity;
}
} // anonymous namespace

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
  bool gravity_compensation =
      controller_config["Gravity_Compensation"].as<bool>();
  std::cout << "Compensation:\n"
            << "Coriolis: " << coriolis_compensation
            << ", Gravity: " << gravity_compensation << std::endl;

  const int ts = controller_config["Sampling_Time"].as<int>();
  const double t_end = controller_config["End_Time"].as<double>();
  std::cout << "Sampling time: " << ts << " ms" << std::endl;

  // Parameters
  const size_t filter_size{1};

  Controllers::PDController pd_controller(filter_size, Kp, Kd,
                                          gravity_compensation);

  try {
    franka::Robot Myrobot(robot_ip);
    franka::Model Mymodel(Myrobot.loadModel());

    Reach_Desired_Joint_Pose(Myrobot);
    Reach_Desired_Joint_Pose(Myrobot, q_desired);
    set_default_collision(Myrobot);

    double t = 0.0;
    int timer = 0;
    franka::RobotState robot_state_sampled = Myrobot.readOnce();

    auto callback_control = [&](const franka::RobotState &robot_state,
                                franka::Duration period) -> franka::Torques {
      // Manual switch sampling frequency
      timer += period.toMSec();
      if (timer % ts == 0) {
        robot_state_sampled = robot_state;
        std::cout << "Sample at " << timer << "ms" << std::endl;
      }
      return pd_controller.step(robot_state_sampled, Mymodel);
    };

    auto callback_motion_generator =
        [&](const franka::RobotState &,
            franka::Duration period) -> franka::JointVelocities {
      t += period.toSec();
      franka::JointVelocities velocities{{0, 0, 0, 0, 0, 0, 0}};
      auto velocity_array = CosineWaveTrajectory(t, amplitudes, omegas);
      for (size_t i = 0; i < 7; i++) {
        velocities.dq[i] = velocity_array[i];
      }
      if (t >= t_end) {
        return franka::MotionFinished(velocities);
      }
      return velocities;
    };

    bool motion_finished = false;
    auto active_control = Myrobot.startJointVelocityControl(
        research_interface::robot::Move::ControllerMode::kExternalController);

    while (!motion_finished) {
      auto read_once_return = active_control->readOnce();
      auto robot_state = read_once_return.first;
      auto duration = read_once_return.second;
      auto joint_velocities = callback_motion_generator(robot_state, duration);
      auto torques = callback_control(robot_state, duration);
      motion_finished = joint_velocities.motion_finished;
      active_control->writeOnce(joint_velocities, torques);
    }

    Reach_Desired_Joint_Pose(Myrobot);

  } catch (const franka::ControlException &e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
