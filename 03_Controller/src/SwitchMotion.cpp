#include <array>
#include <cmath>
#include <iostream>

#include "yaml-cpp/yaml.h"
#include <Eigen/Dense>

#include <franka/active_control.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include "CosineWaveGenerator.h"
#include "DefaultBehavior.h"
#include "EnableArrayPrint.h"
#include "PolynomialMotionGenerator.h"
#include "ReadConfig.h"

#include "matplotlibcpp17/pyplot.h"
#include <vector>

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
  std::array<double, 7> amplitudes =
      controller_config["Amplitude"].as<std::array<double, 7>>();
  std::array<double, 7> omegas =
      controller_config["Omega"].as<std::array<double, 7>>();
  const double t_end = controller_config["End_Time"].as<double>();
  const double t_back = controller_config["Backing_Time"].as<double>();

  try {
    franka::Robot Myrobot(robot_ip);
    Myrobot.automaticErrorRecovery();
    Reach_Desired_Joint_Pose(Myrobot, q_desired);

    franka::RobotState robot_state_1 = Myrobot.readOnce();
    CosineWaveMotionGenerator cosine_motion_generator(
        robot_state_1.q, amplitudes, omegas, t_end);
    // CosineWaveMotionGenerator cosine_motion_generator(q_desired, amplitudes,
    //                                                   omegas, t_end);
    std::array<std::array<double, 7>, 3> kinematics_end_stage_1 =
        cosine_motion_generator.kinematics_end();

    // CubicPolynomialMotionGenerator polynomialmotion_generator(
    //     t_back, kinematics_end_stage_1[0], kinematics_end_stage_1[1],
    //     q_desired);
    Quintic_Polynomial_Motion_Generator polynomialmotion_generator(
        t_back, kinematics_end_stage_1[0], kinematics_end_stage_1[1],
        kinematics_end_stage_1[2], q_desired);

    std::vector<double> time_span;
    std::array<std::vector<double>, 7> q, dq, ddq;
    for (double t = 0.0; t < t_end + t_back; t += 0.01) {
      time_span.push_back(t);
      if (t <= t_end) {
        for (size_t i = 0; i < 7; ++i) {
          q[i].push_back(cosine_motion_generator.trajectory_position(t)[i]);
          dq[i].push_back(cosine_motion_generator.trajectory_velocity(t)[i]);
          ddq[i].push_back(
              cosine_motion_generator.trajectory_acceleration(t)[i]);
        }
      } else {
        for (size_t i = 0; i < 7; ++i) {
          q[i].push_back(
              polynomialmotion_generator.trajectory_position(t - t_end)[i]);
          dq[i].push_back(
              polynomialmotion_generator.trajectory_velocity(t - t_end)[i]);
          ddq[i].push_back(
              polynomialmotion_generator.trajectory_acceleration(t - t_end)[i]);
        }
      }
    }
    pybind11::scoped_interpreter guard{};
    auto plt = matplotlibcpp17::pyplot::import();
    auto [fig, ax] = plt.subplots(
        7, 3,
        Kwargs("figsize"_a = py::make_tuple(22, 15), "tight_layout"_a = true));
    for (int i = 0; i < 7; ++i) {
      ax[3 * i].plot(Args(time_span, q[i]),
                     Kwargs("label"_a = py::str("$q_{}$").format(i + 1)));
      ax[3 * i].axvline(Args(t_end),
                        Kwargs("color"_a = "red", "linestyle"_a = "--",
                               "label"_a = "Switch"));
      ax[3 * i].set_xlabel(Args("$\\mathrm{Time \\enspace (s)}$"),
                           Kwargs("fontsize"_a = 12));
      ax[3 * i].set_ylabel(Args("$\\mathrm{Position \\enspace (rad)}$"),
                           Kwargs("fontsize"_a = 12));
      ax[3 * i].set_title(
          Args(py::str("$q_{}- t \\enspace$ Diagram").format(i + 1)),
          Kwargs("fontsize"_a = 16));
      ax[3 * i].legend();
      ax[3 * i].grid();
      ax[3 * i + 1].plot(
          Args(time_span, dq[i]),
          Kwargs("label"_a = py::str("$\\dot{{q}}_{{{}}}$").format(i + 1)));
      ax[3 * i + 1].axvline(Args(t_end),
                            Kwargs("color"_a = "red", "linestyle"_a = "--",
                                   "label"_a = "Switch"));
      ax[3 * i + 1].set_xlabel(Args("$\\mathrm{Time \\enspace (s)}$"),
                               Kwargs("fontsize"_a = 12));
      ax[3 * i + 1].set_ylabel(Args("$\\mathrm{Velocity \\enspace (rad/s)}$"),
                               Kwargs("fontsize"_a = 12));
      ax[3 * i + 1].set_title(
          Args(py::str("$\\dot{{q}}_{{{}}}- t \\enspace$ Diagram")
                   .format(i + 1)),
          Kwargs("fontsize"_a = 16));
      ax[3 * i + 1].legend();
      ax[3 * i + 1].grid();
      ax[3 * i + 2].plot(
          Args(time_span, ddq[i]),
          Kwargs("label"_a = py::str("$\\ddot{{q}}_{{{}}}$").format(i + 1)));
      ax[3 * i + 2].axvline(Args(t_end),
                            Kwargs("color"_a = "red", "linestyle"_a = "--",
                                   "label"_a = "Switch"));
      ax[3 * i + 2].set_xlabel(Args("$\\mathrm{Time \\enspace (s)}$"),
                               Kwargs("fontsize"_a = 12));
      ax[3 * i + 2].set_ylabel(
          Args("$\\mathrm{Acceleration \\enspace (rad/s^2)}$"),
          Kwargs("fontsize"_a = 12));
      ax[3 * i + 2].set_title(
          Args(py::str("$\\ddot{{q}}_{{{}}}- t \\enspace$ Diagram")
                   .format(i + 1)),
          Kwargs("fontsize"_a = 16));
      ax[3 * i + 2].legend();
      ax[3 * i + 2].grid();
    }
    plt.savefig(Args("./images/joint_trajectory.png"));

    double t = 0.0;
    std::array<double, 7> position;
    auto callback_motion =
        [&](const franka::RobotState &robot_state,
            franka::Duration period) -> franka::JointPositions {
      if (t < t_end) {
        position = cosine_motion_generator.trajectory_position(t);
      } else {
        position = polynomialmotion_generator.trajectory_position(t - t_end);
      }
      t += period.toSec();

      franka::JointPositions position_command{position};
      if (t > t_end + t_back) {
        return franka::MotionFinished(position_command);
      }
      return position_command;
    };

    bool motion_finished = false;
    set_default_collision(Myrobot);
    auto active_control = Myrobot.startJointPositionControl(
        research_interface::robot::Move::ControllerMode::kJointImpedance);
    while (!motion_finished) {
      auto read_once_return = active_control->readOnce();
      auto robot_state = read_once_return.first;
      auto duration = read_once_return.second;
      auto joint_positions = callback_motion(robot_state, duration);
      motion_finished = joint_positions.motion_finished;
      active_control->writeOnce(joint_positions);
    }
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}