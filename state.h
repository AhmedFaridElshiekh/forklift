//
// Created by Ahmed Farid on 15/04/2025.
//

#ifndef STATE_H
#define STATE_H

typedef enum {
  IDLE,
  ROTATE_TO_TARGET,
  MOVE_FORWARD,
  ARRIVED,
  CHARGING,
  LIFT
} RobotState;
#endif // STATE_H
