//
// Created by Ahmed Farid on 18/06/2025.
//
//#include <cstdio>
#include "motors.h"

u32 velocity_to_pwm(f32 velocity, bool* direction)
{
    // 1. Determine direction
    if (velocity < 0)
    {
        // change_pin direction
        *direction = BACKWARD;
    }
    else
    {
        // make the default direction
        *direction = FORWARD;
    }

    // 2. Take absolute value for PWM
    f32 pwm_f = std::fabs(velocity); // Requires #include <math.h>

    // 3. Clamp to max PWM
    if (pwm_f > PWM_MAX)
    {
        pwm_f = PWM_MAX;
    }

    // 4. Cast to int and return
    return (u32)pwm_f;
}

void motors(RobotData* robot_t)
{
    bool direction_r, direction_l;
    robot_t->r_speed_pwm = velocity_to_pwm(robot_t->r_speed, &direction_r);
    robot_t->l_speed_pwm = velocity_to_pwm(robot_t->l_speed, &direction_l);
    // if the speed is small scale it

    // printf("right speed:%f\t", robot_t->r_speed);
    // printf("left speed:%f\t\n", robot_t->l_speed);
    // printf("right speed pwm:%d\t", robot_t->r_speed_pwm);
    // printf("left speed pwn:%d\t\n", robot_t->l_speed_pwm);
    if (direction_r == FORWARD)
    {
        //Motor1_Direction(right_motors,FORWARD);
        // Motor1_Direction(right_motors,BACKWARD);
        // motor_pwm(robot_t->r_speed_pwm)
        return;
    }
    else
    {
        //Motor1_Direction(right_motors,BACKWARD);
        // Motor1_Direction(right_motors,FORWARD);
        // motor_pwm(robot_t->r_speed_pwm)
        return;
    }
    if (direction_l == FORWARD)
    {
        //Motor1_Direction(left_motors,FORWARD);
        // Motor1_Direction(left_motors,BACKWARD);
        // motor_pwm(robot_t->l_speed_pwm)
        return;
    }
    else
    {
        //Motor1_Direction(left_motors,BACKWARD);
        // Motor1_Direction(left_motors,FORWARD);
        // motor_pwm(robot_t->l_speed_pwm)
        return;
    }
}

void stop_motors(RobotData* robot_t)
{
    robot_t->r_speed = 0;
    robot_t->l_speed = 0;
    robot_t->r_speed_pwm = 0;
    robot_t->l_speed_pwm = 0;
    // printf("right speed:%f\t", robot_t->r_speed);
    // printf("left speed:%f\t\n", robot_t->l_speed);
}
