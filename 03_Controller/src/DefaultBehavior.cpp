#include <array>
#include <franka/robot.h>

#include "MotionGenerator.h"

void set_default_collision(franka::Robot &robot,
                           const std::array<double, 7> &torque_lower,
                           const std::array<double, 7> &torque_upper,
                           const std::array<double, 6> &force_lower,
                           const std::array<double, 6> &force_upper) {
  robot.setCollisionBehavior(torque_lower, torque_upper, force_lower,
                             force_upper);
}

void Reach_Desired_Joint_Pose(franka::Robot &robot,
                              const std::array<double, 7> &q_goal,
                              const double &speed_ratio,
                              const std::array<double, 7> &torque_lower,
                              const std::array<double, 7> &torque_upper,
                              const std::array<double, 6> &force_lower,
                              const std::array<double, 6> &force_upper) {
//   franka::RobotState initial_state = robot.readOnce();
  set_default_collision(robot, torque_lower, torque_upper, force_lower,
                        force_upper);
  MotionGenerator motion_generator(q_goal,speed_ratio);
  robot.control(motion_generator);
}