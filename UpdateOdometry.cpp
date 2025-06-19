//
// Created by Ahmed Farid on 21/03/2025.
//

#include "UpdateOdometry.h"

#include "encoder.h"
#include "kinematics.h"


f32 x_RFID = 0.0f, y_RFID = 0.0f;
f32 theta_encoders = 0.0f;
f32 theta = 0;
/*
 * update robot state x ,y , theta
 */
void update_odometry(RobotData* robot_t)
{
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


    //read RFID
    // robot_t->x_current = RFID_read_x;
    // robot_t->y_current = RFID_read_y;

    robot_t->dx = robot_t->x_target - robot_t->x_current;
    robot_t->dy = robot_t->y_target - robot_t->y_current;

    // sensor fusion
}
