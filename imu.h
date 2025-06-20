#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "robot.h"
#include "angles.h"
// float RateRoll, RatePitch, RateYaw;

void gyro_signals(RobotData* robot_t);
void MPU6050_init(void);
f32 angular_to_angle(int16_t angular_speed, f32* old_angle);
#endif  //IMU_H