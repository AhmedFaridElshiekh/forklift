//
// Created by Ahmed Farid on 18/06/2025.
//
#include "motors.h"
#include <Arduino.h>
#include "config.h"

u32 velocity_to_pwm(f32 velocity, bool* direction) {
  // 1. Determine direction
  if (velocity < 0) {
    // change_pin direction
    *direction = BACKWARD;
  } else {
    // make the default direction
    *direction = FORWARD;
  }

  // 2. Take absolute value for PWM
  f32 pwm_f = std::fabs(velocity);  // Requires #include <math.h>

  // 3. Clamp to max PWM
  if (pwm_f > PWM_MAX) {
    pwm_f = PWM_MAX;
  }

  // 4. Cast to int and return
  return (u32)pwm_f;
}

void init_motors() {

  // ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  // ledcAttachPin(MOTOR1_RPWM, 0);
  ledcAttach(MOTOR1_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR1_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR2_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR2_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR3_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR3_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR4_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(MOTOR4_LPWM, PWM_FREQ, PWM_RESOLUTION);
}

void setMotor(uint8_t rpwm, uint8_t lpwm, bool forward, uint8_t speed) {
  if (forward) {
    ledcWrite(rpwm, speed);
    ledcWrite(lpwm, 0);
  } else {
    ledcWrite(rpwm, 0);
    ledcWrite(lpwm, speed);
  }
}
// check for types

void motors(RobotData* robot_t) {
  bool direction_r, direction_l;
  robot_t->r_speed_pwm = velocity_to_pwm(robot_t->r_speed, &direction_r);
  robot_t->l_speed_pwm = velocity_to_pwm(robot_t->l_speed, &direction_l);
  setMotor(MOTOR1_RPWM, MOTOR1_LPWM, direction_r, robot_t->r_speed_pwm);
  setMotor(MOTOR2_RPWM, MOTOR2_LPWM, direction_l, robot_t->l_speed_pwm);
  setMotor(MOTOR3_RPWM, MOTOR3_LPWM, direction_r, robot_t->r_speed_pwm);
  setMotor(MOTOR4_RPWM, MOTOR4_LPWM, direction_l, robot_t->l_speed_pwm);


  // if the speed is small scale it
}

void stop_motors(RobotData* robot_t) {
  setMotor(MOTOR1_RPWM, MOTOR1_LPWM, 0, 0);
  setMotor(MOTOR2_RPWM, MOTOR2_LPWM, 0, 0);
  setMotor(MOTOR3_RPWM, MOTOR3_LPWM, 0, 0);
  setMotor(MOTOR4_RPWM, MOTOR4_LPWM, 0, 0);
}
