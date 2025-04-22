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

int main(int argc, char **argv) {
  // Read config file
  YAML::Node config = GetConfig();
  const std::string robot_ip = config["Robot_ip"].as<std::string>();
  std::array<double, 7> q_zero = config["Zero_Pos"].as<std::array<double, 7>>();
  std::cout << "IP Address of Franka Research 3 Robot: " << robot_ip
            << std::endl;
  std::cout << "Zero Pose: " << q_zero << std::endl;

  try {
    franka::Robot Myrobot(robot_ip);
    Reach_Desired_Joint_Pose(Myrobot);
    Reach_Desired_Joint_Pose(Myrobot,
                             config["Desired_Pos"].as<std::array<double, 7>>());

  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}