#include <iostream>
#include <filesystem>
#include <string>

#include "yaml-cpp/yaml.h"

#include "EnableArrayPrint.h"
#include "ReadConfig.h"
#include "ToEigen.h"

#include <franka/exception.h>
#include <franka/model.h>

int main(int argc, char **argv)
{
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

        franka::Model Mymodel(Myrobot.loadModel());

        // Print Frame and Transformation Matrixes
        int joint_index = 1;
        for (franka::Frame frame = franka::Frame::kJoint1;
             frame <= franka::Frame::kJoint7;
             frame++)
        {
            std::cout << "Frame: " << joint_index << std::endl;
            std::cout << "T_" << joint_index << "_0: " << std::endl;
            std::cout << "From libfranka API, in std::array<double, 16> format: "
                      << std::endl
                      << Mymodel.pose(frame, current_state)
                      << std::endl;
            std::cout << "In Eigen::Matrix4d format: "
                      << std::endl
                      << GetTransMat_4x4(Mymodel.pose(frame, current_state))
                      << std::endl;
            joint_index++;
        }
    }
    catch (franka::Exception const &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}