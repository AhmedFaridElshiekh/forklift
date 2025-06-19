//
// Created by Ahmed Farid on 18/06/2025.
//

#ifndef MOTORS_H
#define MOTORS_H

#include "STD_TYPES.h"
#include "robot.h"
#define MAX_RPM      150.0f      // motor max RPM at full duty
#define PWM_MAX      255        // 8-bit PWM resolution
#define FORWARD      1
#define BACKWARD     0
u32 velocity_to_pwm(f32 velocity, bool* direction);

void motors(RobotData* robot_t);
void stop_motors(RobotData* robot_t);
#endif //MOTORS_H
