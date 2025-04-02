#ifndef DEFAULTBEHAOVIOR_H
#define DEFAULTBEHAOVIOR_H

#include <array>
#include <franka/robot.h>

namespace {
const std::array<double, 7> default_lower_torque_thresholds = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
const std::array<double, 7> default_upper_torque_thresholds = {
    {100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
const std::array<double, 6> default_lower_force_thresholds = {
    {100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
const std::array<double, 6> default_upper_force_thresholds = {
    {100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
} // anonymous namespace to store default values

void set_default_collision(
    franka::Robot &robot,
    const std::array<double, 7> &torque_lower = default_lower_torque_thresholds,
    const std::array<double, 7> &torque_upper = default_upper_torque_thresholds,
    const std::array<double, 6> &force_lower = default_lower_force_thresholds,
    const std::array<double, 6> &force_upper = default_lower_force_thresholds);

#endif