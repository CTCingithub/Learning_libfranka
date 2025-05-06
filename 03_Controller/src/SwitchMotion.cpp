#include <array>
#include <cmath>
#include <iostream>

#include "yaml-cpp/node/node.h"
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

#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"

struct RecordStruct {
  std::mutex mutex;
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 7> tau;
};

void async_log_state(const std::shared_ptr<spdlog::logger> &logger,
                     const std::array<double, 7> &state, std::mutex &mutex) {
  if (mutex.try_lock()) {
    logger->info("{},{},{},{},{},{},{}", state[0], state[1], state[2], state[3],
                 state[4], state[5], state[6]);
    mutex.unlock();
  }
}

void log_state(const std::shared_ptr<spdlog::logger> &logger,
               const std::array<double, 7> &state) {
  logger->info("{},{},{},{},{},{},{}", state[0], state[1], state[2], state[3],
               state[4], state[5], state[6]);
}

void PlotReferenceTraj(
    const std::vector<double> &time_span,
    const std::array<std::vector<double>, 7> &q_ref,
    const std::array<std::vector<double>, 7> &dq_ref,
    const std::array<std::vector<double>, 7> &ddq_ref, const double &t_switch,
    const std::string &file_name = "./images/joint_trajectory.png") {
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();
  auto [fig, ax] = plt.subplots(
      7, 3,
      Kwargs("figsize"_a = py::make_tuple(22, 15), "tight_layout"_a = true));
  for (int i = 0; i < 7; ++i) {
    ax[3 * i].plot(Args(time_span, q_ref[i]),
                   Kwargs("label"_a = py::str("$q_{}$").format(i + 1)));
    ax[3 * i].axvline(
        Args(t_switch),
        Kwargs("color"_a = "red", "linestyle"_a = "--", "label"_a = "Switch"));
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
        Args(time_span, dq_ref[i]),
        Kwargs("label"_a = py::str("$\\dot{{q}}_{{{}}}$").format(i + 1)));
    ax[3 * i + 1].axvline(
        Args(t_switch),
        Kwargs("color"_a = "red", "linestyle"_a = "--", "label"_a = "Switch"));
    ax[3 * i + 1].set_xlabel(Args("$\\mathrm{Time \\enspace (s)}$"),
                             Kwargs("fontsize"_a = 12));
    ax[3 * i + 1].set_ylabel(Args("$\\mathrm{Velocity \\enspace (rad/s)}$"),
                             Kwargs("fontsize"_a = 12));
    ax[3 * i + 1].set_title(
        Args(py::str("$\\dot{{q}}_{{{}}}- t \\enspace$ Diagram").format(i + 1)),
        Kwargs("fontsize"_a = 16));
    ax[3 * i + 1].legend();
    ax[3 * i + 1].grid();
    ax[3 * i + 2].plot(
        Args(time_span, ddq_ref[i]),
        Kwargs("label"_a = py::str("$\\ddot{{q}}_{{{}}}$").format(i + 1)));
    ax[3 * i + 2].axvline(
        Args(t_switch),
        Kwargs("color"_a = "red", "linestyle"_a = "--", "label"_a = "Switch"));
    ax[3 * i + 2].set_xlabel(Args("$\\mathrm{Time \\enspace (s)}$"),
                             Kwargs("fontsize"_a = 12));
    ax[3 * i + 2].set_ylabel(
        Args("$\\mathrm{Acceleration \\enspace (rad/s^2)}$"),
        Kwargs("fontsize"_a = 12));
    ax[3 * i + 2].set_title(
        Args(
            py::str("$\\ddot{{q}}_{{{}}}- t \\enspace$ Diagram").format(i + 1)),
        Kwargs("fontsize"_a = 16));
    ax[3 * i + 2].legend();
    ax[3 * i + 2].grid();
  }
  plt.savefig(Args("./images/joint_trajectory.png"));
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
  std::array<double, 7> amplitudes =
      controller_config["Amplitude"].as<std::array<double, 7>>();
  std::array<double, 7> omegas =
      controller_config["Omega"].as<std::array<double, 7>>();
  const double t_switch = controller_config["Switch_Time"].as<double>();
  const double t_back = controller_config["Backing_Time"].as<double>();

  // Logger
  spdlog::init_thread_pool(32768, 1);
  auto async_logger_q = spdlog::create_async<spdlog::sinks::basic_file_sink_mt>(
      "async_logger_q", "./log/async_logger_q.csv", true);
  auto async_logger_dq =
      spdlog::create_async<spdlog::sinks::basic_file_sink_mt>(
          "async_logger_dq", "./log/async_logger_dq.csv", true);
  auto async_logger_tau =
      spdlog::create_async<spdlog::sinks::basic_file_sink_mt>(
          "async_logger_tau", "./log/async_logger_tau.csv", true);
  async_logger_q->set_pattern("%v");
  async_logger_dq->set_pattern("%v");
  async_logger_tau->set_pattern("%v");
  async_logger_q->info("time,q_1,q_2,q_3,q_4,q_5,q_6,q_7");
  async_logger_q->flush();
  async_logger_dq->info("time,dq_1,dq_2,dq_3,dq_4,dq_5,dq_6,dq_7");
  async_logger_dq->flush();
  async_logger_tau->info("time,tau_1,tau_2,tau_3,tau_4,tau_5,tau_6,tau_7");
  async_logger_tau->flush();
  async_logger_q->set_pattern("%Y-%m-%d %H:%M:%S.%f,%v");
  async_logger_dq->set_pattern("%Y-%m-%d %H:%M:%S.%f,%v");
  async_logger_tau->set_pattern("%Y-%m-%d %H:%M:%S.%f,%v");
  RecordStruct Record;

  try {
    franka::Robot Myrobot(robot_ip);
    Myrobot.automaticErrorRecovery();
    Reach_Desired_Joint_Pose(Myrobot, q_desired);

    franka::RobotState robot_state_1 = Myrobot.readOnce();
    CosineWaveMotionGenerator cosine_motion_generator(
        robot_state_1.q, amplitudes, omegas, t_switch);

    std::array<std::array<double, 7>, 3> kinematics_end_stage_1 =
        cosine_motion_generator.kinematics_end();

    Quintic_Polynomial_Motion_Generator polynomialmotion_generator(
        t_back, kinematics_end_stage_1[0], kinematics_end_stage_1[1],
        kinematics_end_stage_1[2], q_desired);

    std::vector<double> time_span;
    std::array<std::vector<double>, 7> q_traj, dq_traj, ddq_traj;
    for (double t = 0.0; t < t_switch + t_back; t += 0.01) {
      time_span.push_back(t);
      if (t <= t_switch) {
        for (size_t i = 0; i < 7; ++i) {
          q_traj[i].push_back(
              cosine_motion_generator.trajectory_position(t)[i]);
          dq_traj[i].push_back(
              cosine_motion_generator.trajectory_velocity(t)[i]);
          ddq_traj[i].push_back(
              cosine_motion_generator.trajectory_acceleration(t)[i]);
        }
      } else {
        for (size_t i = 0; i < 7; ++i) {
          q_traj[i].push_back(
              polynomialmotion_generator.trajectory_position(t - t_switch)[i]);
          dq_traj[i].push_back(
              polynomialmotion_generator.trajectory_velocity(t - t_switch)[i]);
          ddq_traj[i].push_back(
              polynomialmotion_generator.trajectory_acceleration(t -
                                                                 t_switch)[i]);
        }
      }
    }

    std::cout << "Start Plotting with matplotlibcpp17" << std::endl;
    PlotReferenceTraj(time_span, q_traj, dq_traj, ddq_traj, t_switch);
    std::cout << "Saved figure" << std::endl;

    Myrobot.automaticErrorRecovery();

    double t = 0.0;
    std::array<double, 7> position;
    Record.q = robot_state_1.q;
    Record.dq = robot_state_1.dq;
    Record.tau = robot_state_1.tau_J;

    std::atomic_bool is_logging{true};

    auto callback_motion =
        [&](const franka::RobotState &robot_state,
            franka::Duration period) -> franka::JointPositions {
      if (t < t_switch) {
        position = cosine_motion_generator.trajectory_position(t);
        std::lock_guard<std::mutex> lock(Record.mutex);
        Record.q = robot_state.q;
        Record.dq = robot_state.dq;
        Record.tau = robot_state.tau_J;
      } else {
        is_logging = false;
        position = polynomialmotion_generator.trajectory_position(t -
        t_switch);
      }
      t += period.toSec();

      franka::JointPositions position_command{position};
      if (t > t_switch + t_back) {
        return franka::MotionFinished(position_command);
      }
      return position_command;
    };

    // auto callback_motion =
    //     [&](const franka::RobotState &robot_state,
    //         franka::Duration period) -> franka::JointPositions {
    //   if (t > t_switch) {
    //     is_logging = false;
    //   }
    //   std::cout << "t: " << t << " ms" << std::endl;
    //   for (size_t i = 0; i < 7; i++) {
    //     position[i] = q_traj[i][t];
    //   }
    //   franka::JointPositions position_command{position};
    //   t += period.toMSec();
    //   if (t > t_switch + t_back) {
    //     return franka::MotionFinished(position_command);
    //   }
    //   return position_command;
    // };

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
      if (is_logging) {
        // async_log_state(async_logger_q, Record.q, Record.mutex);
        // async_log_state(async_logger_dq, Record.dq, Record.mutex);
        // async_log_state(async_logger_tau, Record.tau, Record.mutex);
        log_state(async_logger_q, Record.q);
        log_state(async_logger_dq, Record.dq);
        log_state(async_logger_tau, Record.tau);
      }
    }
  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  spdlog::shutdown();

  return 0;
}