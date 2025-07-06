//
// Created by Ahmed Farid on 08/03/2025.
//

#ifndef PID_H
#define PID_H
#include "STD_TYPES.h"

// robot constrains
#define MAX_ANGULAR_VELOCITY 10.0f /* Max angular velocity  (rad/s) */
#define MAX_LINEAR_VELOCITY 10.0f  /* Max motor speed (m/s) */

// PID parameters
#define KV 0.5f /* Gain for linear velocity control */
#define KP_HEADING 3.0f /* PID gains for heading control */
#define KI_HEADING 5.0f
#define KD_HEADING 4.0f
#define TAU .5f
#define PID_LIM_MIN_INT 3.0f
#define PID_LIM_MAX_INT 3.0f

#define DT .0001f /* Sampling time */
#define DESIRED_DISTANCE 1.0f
#define HEADING_C 1
#define DISTANCE_C 0

typedef struct {
  /* Controller gains */
  f32 Kp;
  f32 Ki;
  f32 Kd;

  /* Derivative low-pass filter time constant */
  f32 tau;

  /* Output limits */
  f32 limMin;
  f32 limMax;

  /* Integrator limits */
  f32 limMinInt;
  f32 limMaxInt;

  /* Sample time (in seconds) */
  f32 T;

  /* Controller "memory" */
  f32 integrator;
  f32 prevError; /* Required for integrator */
  f32 differentiator;
  f32 prevMeasurement; /* Required for differentiator */

  /* Controller output */
  f32 out;
  u8 flag; // theta or distance error 0 distance 1 angle
} PIDController;

void PIDController_Init(PIDController *pid);
f32 PIDController_Update(PIDController *pid, f32 setpoint, f32 measurement);
#endif // PID_H
