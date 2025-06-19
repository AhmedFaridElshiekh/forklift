//
// Created by Ahmed Farid on 18/03/2025.
//
#include "kinematics.h"
#include "fsm.h"
/*
* Converts the desired linear and angular velocities from unicycle model to differential drive format.
 */
void uni_to_diff(f32 desired_angular_velocity, f32 desired_linear_velocity, RobotData* robot_t)
{
    robot_t->r_speed =
        (2 * desired_linear_velocity + (desired_angular_velocity * L)) / (2.0f * WHEEL_RADIUS);
    robot_t->l_speed =
        (2 * desired_linear_velocity - (desired_angular_velocity * L)) / (2.0f * WHEEL_RADIUS);
    // check if we need to check for max
}
