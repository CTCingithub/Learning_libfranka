#include <iostream>
#include <filesystem>
#include <string>

#include "yaml-cpp/yaml.h"

#include "MotionGenerator.h"
#include "EnableArrayPrint.h"
#include "ReadConfig.h"
#include "ToEigen.h"

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/duration.h>

int main(int argc, char **argv)
{
    YAML::Node config = GetConfig();
    const std::string robot_ip = config["Robot_ip"].as<std::string>();
    std::array<double, 7> q_target = config["Zero_Pos"].as<std::array<double, 7>>();
    std::cout << "IP Address of Franka Research 3 Robot: " << robot_ip << std::endl;
    std::cout << "q_target: " << q_target << std::endl;

    try
    {
        franka::Robot Myrobot(robot_ip);
        franka::RobotState current_state = Myrobot.readOnce();
        franka::Model Mymodel(Myrobot.loadModel());

        MotionGenerator motion_generator(q_target);

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

        std::cout << "Current Mass Matrix:\n"
                  << GetMat_7x7(Mymodel.mass(current_state)) << std::endl;
        std::cout << "Current Coriolis Vector:\n"
                  << Mymodel.coriolis(current_state) << std::endl;
        std::cout << "Current Gravity Vector:\n"
                  << Mymodel.gravity(current_state) << std::endl;
        std::cout << "Current Torques:\n"
                  << current_state.tau_J << std::endl;

        // Compliance parameters
        const double translational_stiffness{150.0};
        const double rotational_stiffness{10.0};
        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                           Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                               Eigen::MatrixXd::Identity(3, 3);
        // equilibrium point is the initial position
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(current_state.O_T_EE.data()));
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.rotation());

        // set collision behavior
        Myrobot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                     {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                     {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                     {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
            constant_torque_callback = [&](const franka::RobotState &robot_state,
                                           franka::Duration) -> franka::Torques
        {
            // Print Read Joint Angle(degree)
            std::cout << "Current Joint Angle(degree): [";
            for (const auto &joint : robot_state.q)
            {
                std::cout << joint / M_PI * 180;
                if (&joint != &robot_state.q.back())
                {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
            // get state variables
            std::array<double, 7> coriolis_array = Mymodel.coriolis(robot_state);
            std::array<double, 42> jacobian_array =
                Mymodel.zeroJacobian(franka::Frame::kEndEffector, robot_state);

            // convert to Eigen
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.rotation());

            // compute error to desired equilibrium pose
            // position error
            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << position - position_d;

            // orientation error
            // "difference" quaternion
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
            {
                orientation.coeffs() << -orientation.coeffs();
            }
            // "difference" quaternion
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            // Transform to base frame
            error.tail(3) << -transform.rotation() * error.tail(3);

            // compute control
            Eigen::VectorXd tau_task(7), tau_d(7);

            // Spring damper system with damping ratio=1
            tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            tau_d << tau_task + coriolis;

            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
            return tau_d_array;
        };
        Myrobot.control(constant_torque_callback);
    }
    catch (franka::Exception const &e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}