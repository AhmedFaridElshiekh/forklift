//
// Created by Ahmed Farid on 18/06/2025.
//
#include "angles.h"
#include "config.h"
#include "motors.h"
#include <Arduino.h>

void init_motors() {

  ledcAttachPin(MOTOR1_RPWM, rrpwm_motor_channel);
  ledcAttachPin(MOTOR1_LPWM, rlpwm_motor_channel);
  ledcAttachPin(MOTOR2_RPWM, lrpwm_motor_channel);
  ledcAttachPin(MOTOR2_LPWM, llpwm_motor_channel);

  //  ledcAttachPin(LIFT_RPWM, lift_rpwm_motor_channel);
  // ledcAttachPin(LIFT_LPWM, lift_lpwm_motor_channel);

  ledcSetup(rrpwm_motor_channel, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(rlpwm_motor_channel, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(lrpwm_motor_channel, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(llpwm_motor_channel, PWM_FREQ, PWM_RESOLUTION);

  // ledcSetup(lift_rpwm_motor_channel, PWM_FREQ, PWM_RESOLUTION);
  // ledcSetup(lift_lpwm_motor_channel, PWM_FREQ, PWM_RESOLUTION);




}

void setMotor(uint8_t rpwm_channel, uint8_t lpwm_channel, bool forward,
              uint8_t speed) {
  Serial.print("motor rpm : ");
  Serial.println(speed);
  Serial.print("direction :");
  Serial.println(forward);
  if (forward) {
    ledcWrite(rpwm_channel, speed);
    ledcWrite(lpwm_channel, 0);
  } else {
    ledcWrite(rpwm_channel, 0);
    ledcWrite(lpwm_channel, speed);
  }
}
// check for types

u32 velocity_to_pwm(f32 velocity, bool *direction) {
  // 1. Determine direction
  if (velocity < 0) {
    // change_pin direction
    *direction = BACKWARD;
  } else {
    // make the default direction
    *direction = FORWARD;
  }

  // 2. Take absolute value for PWM
  f32 pwm_f = std::fabs(velocity); // Requires #include <math.h>

  // 3. Clamp to max PWM
  if (pwm_f > PWM_MAX) {
    pwm_f = PWM_MAX;
  }

  // 4. Cast to int and return
  return (u32)pwm_f;
}

void motors(RobotData *robot_t) {
  bool direction_r, direction_l;
  robot_t->r_speed_pwm = velocity_to_pwm(robot_t->r_speed, &direction_r);
  robot_t->l_speed_pwm = velocity_to_pwm(robot_t->l_speed, &direction_l);
  // ////////////////////////////////////////////////////
  // // test without pid

  // f32 desired_angle = compute_desired_angle(robot_t);
  // f32 error_angle = desired_angle - robot_t->gyro[2];
  // // limit the angle for [-pi,pi]
  // error_angle = normalize_angle(error_angle);

  // if (robot_t->state == ROTATE_TO_TARGET) {
  //   if (error_angle > 0) {
  //     direction_r = FORWARD;
  //     direction_l = BACKWARD;
  //     robot_t->r_speed_pwm = 155, robot_t->l_speed_pwm = 155;
  //   } else {
  //     direction_r = BACKWARD;
  //     direction_l = FORWARD;
  //     robot_t->r_speed_pwm = 155, robot_t->l_speed_pwm = 155;
  //   }
  // }
  // if (robot_t->state == MOVE_FORWARD) {
  //   direction_r = FORWARD;
  //   direction_l = FORWARD;
  //   robot_t->r_speed_pwm = 250;
  //   robot_t->l_speed_pwm = 250; 
  // }

  ///////////////////////////////////////
  setMotor(rrpwm_motor_channel, rlpwm_motor_channel, direction_r,
           robot_t->r_speed_pwm);
  setMotor(lrpwm_motor_channel, llpwm_motor_channel, direction_l,
           robot_t->l_speed_pwm);
  Serial.print("rspeed: ");
  Serial.print(robot_t->r_speed);
  Serial.print(" lspeed: ");
  Serial.println(robot_t->l_speed);

  // if the speed is small scale it
}

void stop_motors(RobotData *robot_t) {
  setMotor(rrpwm_motor_channel, rlpwm_motor_channel, 0, 0);
  setMotor(lrpwm_motor_channel, llpwm_motor_channel, 0, 0);
}

void fork(RobotData *robot_t){
  if (robot_t->fork_state == NORMAL) {
    setMotor(rlpwm_motor_channel, llpwm_motor_channel, FORWARD, 100);
  } else if (robot_t->fork_state == UP) {
    setMotor(rlpwm_motor_channel, llpwm_motor_channel, FORWARD, 200);
    setMotor(rrpwm_motor_channel, rlpwm_motor_channel, FORWARD, 200);
  } else if (robot_t->fork_state == DOWN) {
    setMotor(rlpwm_motor_channel, llpwm_motor_channel, BACKWARD, 50);
    setMotor(rrpwm_motor_channel, rlpwm_motor_channel, BACKWARD, 50);
  }
}
