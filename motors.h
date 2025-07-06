//
// Created by Ahmed Farid on 18/06/2025.
//

#ifndef MOTORS_H
#define MOTORS_H

#include "STD_TYPES.h"
#include "robot.h"
#define FORWARD 1
#define BACKWARD 0

#define PWM_MAX 255

u32 velocity_to_pwm(f32 velocity, bool *direction);
void init_motors();
void motors(RobotData *robot_t);
void setMotor(uint8_t rpwm, uint8_t lpwm, bool forward, uint8_t speed);
void stop_motors(RobotData *robot_t);
void fork(RobotData *robot_t);
#endif // MOTORS_H
