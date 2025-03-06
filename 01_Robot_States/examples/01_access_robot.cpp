#include <iostream>
#include <filesystem>
#include <string>
#include <typeinfo>

#include "yaml-cpp/yaml.h"
#include "EnableArrayPrint.h"

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char **argv)
{
    // Load Configuration
    std::string file = std::filesystem::current_path().parent_path().string() + "/config/FR3.yaml";
    YAML::Node config = YAML::LoadFile(file);
    const std::string robot_ip = config["Robot_ip"].as<std::string>();
    std::cout << robot_ip << std::endl;

    try
    {
        franka::Robot Myrobot(robot_ip);
        franka::RobotState current_state = Myrobot.readOnce();

        std::cout << "Current Joint Angle(rad): "
                  << current_state.q << std::endl;
    }
    catch (franka::Exception const &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}