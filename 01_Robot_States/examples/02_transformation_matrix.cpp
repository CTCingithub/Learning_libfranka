#include <iostream>
#include <filesystem>
#include <string>

#include "yaml-cpp/yaml.h"
#include "ReadConfig.h"

#include <franka/exception.h>
#include <franka/robot.h>

int main(int argc, char **argv)
{
    // Load Configuration
    // std::string file = std::filesystem::current_path().parent_path().string() + "/config/FR3.yaml";
    // YAML::Node config = YAML::LoadFile(file);
    YAML::Node config = GetConfig();
    const std::string robot_ip = config["Robot_ip"].as<std::string>();
    std::cout << "IP Address of Franka Research 3 Robot: " << robot_ip << std::endl;

    try
    {
        franka::Robot Myrobot(robot_ip);
        franka::RobotState current_state = Myrobot.readOnce();

        // Print Read Joint Angle(degree)
        std::cout << "Read Joint Angle(degree): [";
        for (const auto &joint : current_state.q)
        {
            std::cout << joint / M_PI * 180;
            if (&joint != &current_state.q.back())
            {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }
    catch (franka::Exception const &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}