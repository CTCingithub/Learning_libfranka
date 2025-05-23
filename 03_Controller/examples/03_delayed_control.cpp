#include <array>
#include <bits/std_function.h> // #include <functional>
#include <cmath>
#include <iostream>

#include "yaml-cpp/node/node.h" // #include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>

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

int constant_delay_time(const double &time) {
  // Input in seconds
  // Output in milliseconds
  return 2;
}
int sin_delay_time(const double &time) {
  // Input in seconds
  // Output in milliseconds
  double delay_second = 0.005 * (sin(2 * M_1_PI / 0.005 * time) + 1);
  return static_cast<int>(delay_second * 1000);
}

template <typename T>
T get_delayed_state(boost::circular_buffer<T> &buffer, const int &delay_time) {
  // Take t_delay = 3ms, buffer.size = 6 as example
  // For t \in [0,3]ms, state_delayed is state at 0ms, which is buffer.front()
  // If t = 4ms, buffer is {0,1,2,3,4}, state_delayed is state at 1ms, which is
  // buffer[buffer.size()-1-3] If t = 5ms, buffer is {0,1,2,3,4,5},
  // state_delayed is state at 2ms, which is buffer[buffer.size()-1-3] If t =
  // 6ms, buffer is {1,2,3,4,5,6}, state_delayed is state at 3ms, which is
  // buffer[buffer.size()-1-3] If t = 7ms, buffer is {2,3,4,5,6,7},
  // state_delayed is state at 4ms, which is buffer[buffer.size()-1-3]
  if (delay_time > buffer.size()) {
    return buffer.front();
  } else {
    return buffer[buffer.size() - 1 - delay_time];
  }
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
  std::array<double, 7> Kp =
      controller_config["Kp"].as<std::array<double, 7>>();
  std::array<double, 7> Kd =
      controller_config["Kd"].as<std::array<double, 7>>();
  std::array<double, 7> amplitudes =
      controller_config["Amplitude"].as<std::array<double, 7>>();
  std::array<double, 7> omegas =
      controller_config["Omega"].as<std::array<double, 7>>();
  const double t_switch = controller_config["Switch_Time"].as<double>();

  // Create circular buffer to store delayed states
  int max_delay = controller_config["Max_Delay_Time"].as<int>();
  boost::circular_buffer<double> t_buffer(max_delay);
  boost::circular_buffer<franka::RobotState> robot_state_buffer(max_delay);

  // Controller
  Controllers::PDController pd_controller(Kp, Kd);

  try {
    franka::Robot Myrobot(robot_ip);
    franka::Model Mymodel(Myrobot.loadModel());
    Myrobot.automaticErrorRecovery();

    Reach_Desired_Joint_Pose(Myrobot);
    Reach_Desired_Joint_Pose(Myrobot, q_desired);
    set_default_collision(Myrobot);

    double t = 0.0;
    std::cout << std::fixed << std::setprecision(3);
    t_buffer.push_back(t);
    robot_state_buffer.push_back(Myrobot.readOnce());

    auto callback_control_torque =
        [&](const franka::RobotState &robot_state,
            franka::Duration period) -> franka::Torques {
      t += period.toSec();
      t_buffer.push_back(t);
      auto delay = sin_delay_time(t); // time-varying delay
      std::cout << "current time is " << t << "s, using state of "
                << get_delayed_state(t_buffer, delay) << "s, delay is "
                << delay / 1000.0 << "s" << std::endl;
      robot_state_buffer.push_back(robot_state);
      auto robot_state_delay = get_delayed_state(robot_state_buffer, delay);
      auto torque = pd_controller.step(robot_state_delay, Mymodel);
      return torque;
    };

    CosineWaveMotionGenerator motion_generator(Myrobot.readOnce().q, amplitudes,
                                               omegas, t_switch);
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
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
