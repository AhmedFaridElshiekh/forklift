//
// Created by Ahmed Farid on 21/03/2025.
//

#ifndef ANGLES_H
#define ANGLES_H
#include "STD_TYPES.h"
#define TOLERANCEANGLE 0.0523598776f //radian
#include "robot.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_TWOPI
#define M_TWOPI 6.28318530717958647692f
#endif


f32 abs_t(f32 value);
f32 compute_desired_angle(RobotData* robot_t);
f32 normalize_angle(f32 theta);
u8 has_reached_target(RobotData* robot_t);


#endif //ANGLES_H
