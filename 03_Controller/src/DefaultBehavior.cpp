#include <franka/robot.h>
#include <array>

void set_default_collision(franka::Robot &robot,
                           const std::array<double, 7> &torque_lower,
                           const std::array<double, 7> &torque_upper,
                           const std::array<double, 6> &force_lower,
                           const std::array<double, 6> &force_upper) {
  robot.setCollisionBehavior(torque_lower, torque_upper, force_lower,
                             force_upper);
}