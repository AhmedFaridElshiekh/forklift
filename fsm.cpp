#include "HardwareSerial.h"
#include "esp32-hal-gpio.h"
#include "fsm.h"
#include "UpdateOdometry.h"
#include "angles.h"
#include "compute_velocity.h"
#include "motors.h"
#include "pid.h"
#include <Arduino.h>
#include "config.h"
u8 static current_waypoint = 0;

/*
 * fsm_update updates the robot state machine based on the current state and
 * target points.
 */

void fsm_update(RobotData *robot_t, f32 points[][2], size_t num_points) {
  Serial.print("state: ");
  Serial.println(robot_t->state);

  // if (robot_t->battery_state == BATTERY_CRITICAL) {
  //   robot_t->state = CHARGING;
  // }

  update_odometry(robot_t);  // update robot x,y,theta
  f32 desired_angle = compute_desired_angle(robot_t);
  f32 error_angle = desired_angle - robot_t->gyro[2];
  // limit the angle for [-pi,pi]
  error_angle = normalize_angle(error_angle);
  switch (robot_t->state) {
    case IDLE:
      stop_motors(robot_t);
      set_target(robot_t, points, num_points);
      break;

    case ROTATE_TO_TARGET:
      if (std::fabs(error_angle) <= TOLERANCEANGLE) {
        stop_motors(robot_t);
        robot_t->state = MOVE_FORWARD;
        // robot_t->distance_encoders = 0.0f; // Reset for new segment
        PIDController_Init(&robot_t->pid_heading);
      } else {
        compute_velocity(desired_angle, DESIRED_DISTANCE, robot_t);
        motors(robot_t);
      }

      break;

    case MOVE_FORWARD:

      if (fabs(robot_t->x_target - points[num_points - 1][0]) < 0.2 && fabs(robot_t->y_target - points[num_points - 1][1]) < 0.2) {
        robot_t->state = ARRIVED;
      }


      if (has_reached_target(robot_t)) {
        stop_motors(robot_t);
        robot_t->state = ARRIVED;
        // PIDController_Init(&robot_t->pid_linear); // update it like arrived
        // here is turning point
      } else {
        compute_velocity(desired_angle, DESIRED_DISTANCE, robot_t);
        motors(robot_t);
      }
      break;


    case ARRIVED:
      if (fabs(robot_t->x_target - points[num_points - 1][0]) < 0.2 && fabs(robot_t->y_target - points[num_points - 1][1]) < 0.2) {
        current_waypoint = 0;
        PIDController_Init(&robot_t->pid_heading);  // check if needed
        robot_t->distance_encoders = 0.0f;
        robot_t->state = LIFT;
        // request new root and lift
      } else {
        set_target(robot_t, points, num_points);
      }

      break;
    case LIFT:
      // compute_velocity(desired_angle, .2, robot_t);
      // robot_t->fork_state = DOWN;
      // fork(robot_t);
      // if (digitalRead(LIMIT_SWITCH_DOWN) == 1) {
      //   // if the limit switch is pressed
      //   // keep the same tension
      //   if (digitalRead(LIMIT_SWITCH_FORK) == 1) {
      //     robot_t->fork_state = UP;
      //     fork(robot_t);
      //   } 
      //   else {
      //     //setMotor()
      //     Serial.print("set motors to specif speed until it reaches the card");
      //   }
      // } 
      // else if (digitalRead(LIMIT_SWITCH_UP) == 1) {
      //   robot_t->fork_state = LIFTING;
      // }
      // if (robot_t->fork_state == LIFTING && has_reached_target(robot_t)) {

      //   // get new data
      //   // set_target(robot_t, points, num_points);
      //   robot_t->state = ROTATE_TO_TARGET;
      // }

      break;

    case CHARGING:
      // send to the app it will go charging and the app shall allocate its tasks
      // to a different robot get an array from the app with the path
      //   f32 f[][2] = {
      //     { 0, 1 },  // (0.0, 1.0m)
      //     { 1, 1 },  // (2.0m, 1.0m)
      //     { 1, 0 },  // (2.0m, 0.0)
      //     { 2, 0 }   // (3.0m, 0.0)

      //   };
      //   size_t n = sizeof(f) / sizeof(f[0]);

      //   set_target(robot_t, f, n);
      break;
  }
}

/*
 * set_target sets the target coordinates for the robot based on did it reach
 * the goal.
 */
void set_target(RobotData *robot_t, f32 points[][2], size_t num_points) {
  if (robot_t->state == IDLE || robot_t->state == CHARGING || robot_t->state == ARRIVED) {
    if (current_waypoint >= num_points)
      return;

    robot_t->x_target = points[current_waypoint][0];
    robot_t->y_target = points[current_waypoint][1];
    robot_t->state = ROTATE_TO_TARGET;
    current_waypoint++;
  }
}
