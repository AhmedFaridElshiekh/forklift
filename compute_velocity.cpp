//
// Created by Ahmed Farid on 15/04/2025.
//

#include "compute_velocity.h"
#include "kinematics.h"
#include "pid.h"
#include <Arduino.h>
/*
 * * Computes the desired linear and angular velocities for the robot based on
 * its current state. If the robot is in the MOVE_FORWARD state, it calculates
 * both linear and angular velocities. If it is in the ROTATE_TO_TARGET state,
 * it only calculates the angular velocity and set linear to zero. The computed
 * velocities are then converted from unicycle to differential drive format.
 *
 * set_point_angle The desired angle to reach (in radians).
 *  set_point_distance The desired distance to travel (in meters).
 *  robot_t Pointer to the RobotData structure containing robot state and
 * parameters.
 */

void compute_velocity(f32 set_point_angle, f32 set_point_distance,
                      RobotData *robot_t) {
  f32 desired_linear_velocity;
  f32 desired_angular_velocity;

  if (robot_t->state == MOVE_FORWARD) {
    desired_angular_velocity = PIDController_Update(
        &robot_t->pid_heading, set_point_angle, robot_t->gyro[2]);
    desired_linear_velocity = KV*(set_point_distance - robot_t->distance_encoders)*std::cos(robot_t->gyro[2]);
    Serial.print("liear desired: ");
    Serial.print(desired_linear_velocity);
    Serial.print(" angluar desired: ");
    Serial.println(desired_angular_velocity);

    uni_to_diff(desired_angular_velocity, desired_linear_velocity, robot_t);

  }
   else if (robot_t->state == ROTATE_TO_TARGET) {
    desired_angular_velocity = PIDController_Update(
        &robot_t->pid_heading, set_point_angle, robot_t->gyro[2]);
    uni_to_diff(desired_angular_velocity, 0.0f, robot_t);
  }
}
