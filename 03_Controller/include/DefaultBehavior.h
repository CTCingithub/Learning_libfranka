#ifndef DEFAULTBEHAOVIOR_H
#define DEFAULTBEHAOVIOR_H

#include <array>
#include <franka/robot.h>

#include "MotionGenerator.h"

namespace {
const std::array<double, 7> default_lower_torque_thresholds = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
const std::array<double, 7> default_upper_torque_thresholds = {
    {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
const std::array<double, 6> default_lower_force_thresholds = {
    {100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
const std::array<double, 6> default_upper_force_thresholds = {
    {100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
const std::array<double, 7> zero_pose = {
    {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
} // namespace

void set_default_collision(
    franka::Robot &robot,
    const std::array<double, 7> &torque_lower = default_lower_torque_thresholds,
    const std::array<double, 7> &torque_upper = default_upper_torque_thresholds,
    const std::array<double, 6> &force_lower = default_lower_force_thresholds,
    const std::array<double, 6> &force_upper = default_lower_force_thresholds);

void Reach_Desired_Joint_Pose(
    franka::Robot &robot, const std::array<double, 7> &q_goal = zero_pose,
    const double &speed_ratio = 0.1,
    const std::array<double, 7> &torque_lower = default_lower_torque_thresholds,
    const std::array<double, 7> &torque_upper = default_upper_torque_thresholds,
    const std::array<double, 6> &force_lower = default_lower_force_thresholds,
    const std::array<double, 6> &force_upper = default_lower_force_thresholds);

#endif