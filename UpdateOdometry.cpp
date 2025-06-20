//
// Created by Ahmed Farid on 21/03/2025.
//

#include "UpdateOdometry.h"

#include "encoder.h"
#include "kinematics.h"
#include "rfid.h"


f32 theta_encoders = 0.0f;
f32 theta = 0;
byte RFID_x_old = 0, RFID_y_old = 0;
/*
 * update robot state x ,y , theta
 */
void update_odometry(RobotData* robot_t) {
  f32 distance_center = (get_left_distance() + get_right_distance()) / 2;
  f32 x_dt = distance_center * std::cos(theta);
  f32 y_dt = distance_center * std::sin(theta);
  f32 theta_dt = (get_right_distance() - get_left_distance()) / L;
  robot_t->x_current += x_dt;
  robot_t->y_current += y_dt;
  theta_encoders += theta_dt;
  // gyroupdate(robot_t);
  theta = robot_t->gyro[2];
  robot_t->distance_encoders += distance_center;
  byte RFID_read_x;
  byte RFID_read_y;

  bool state = readXYFromCard(RFID_read_x, RFID_read_y);
  if (RFID_x_old != RFID_read_x && RFID_y_old != RFID_read_y && state) {
    robot_t->x_current =(f32) RFID_read_x;
    robot_t->y_current = (f32)RFID_read_y;
    RFID_x_old = RFID_read_x;
    RFID_y_old = RFID_read_y;
  }


  robot_t->dx = robot_t->x_target - robot_t->x_current;
  robot_t->dy = robot_t->y_target - robot_t->y_current;

  // sensor fusion
}
