//
// Created by Ahmed Farid on 15/04/2025.
//

#ifndef ROBOT_H
#define ROBOT_H
#include "STD_TYPES.h"
#include "pid.h"
#include "state.h"
#include <cmath>

typedef enum {
  BATTERY_NORMAL,  // >50%
  BATTERY_LOW,     // 20-50%
  BATTERY_CRITICAL // <20%
} BatteryState;
typedef enum {
  NORMAL, 
  DOWN, 
  UP,
  LIFTING
} ForkState;

typedef struct {
  // Position data
  f32 x_current, y_current;
  f32 x_target, y_target;
  f32 dx, dy;
  f32 r_speed, l_speed;
  u32 r_speed_pwm, l_speed_pwm;

  // Sensor data
  f32 gyro[3];
  f32 accel[3];
  f32 distance_encoders;
  // System state
  RobotState state;
  BatteryState battery_state;
  u8 ID;

  // Control data
  PIDController pid_heading;
  ForkState fork_state;
} RobotData;
#endif // ROBOT_H
