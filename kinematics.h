//
// Created by Ahmed Farid on 18/03/2025.
//

#ifndef KINEMATICS_H
#define KINEMATICS_H
#include "STD_TYPES.h"
#include "robot.h"
#define WHEEL_RADIUS 1.0f
#define L 1.0f
void uni_to_diff(f32 desired_angular_velocity, f32 desired_linear_velocity, RobotData* robot_t);
#endif //KINEMATICS_H
