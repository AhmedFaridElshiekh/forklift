//
// Created by Ahmed Farid on 08/03/2025.
//

#ifndef FSM_H
#define FSM_H
#include "STD_TYPES.h"
#include "robot.h"

void fsm_update(RobotData* robot_t, f32 points[][2], size_t num_points);

void set_target(RobotData* robot_t, f32 points[][2], size_t num_points);

#endif
