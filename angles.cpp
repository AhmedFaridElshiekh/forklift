//
// Created by Ahmed Farid on 21/03/2025.
//
#include "fsm.h"
#include "angles.h"

u8 has_reached_target(RobotData* robot_t)
{
    return (std::fabs(robot_t->dx) < .01f) && (std::fabs(robot_t->dy) < .01f);
}

f32 compute_desired_angle(RobotData* robot_t)
{
    if (has_reached_target(robot_t))
    {
        robot_t->state = ARRIVED;
        return 0.0f;
    }

    // Determine which movement component is dominant.
    if (std::fabs(robot_t->dx) >= std::fabs(robot_t->dy))
    {
        // Horizontal movement is dominant.
        if (robot_t->dx > 0.0f)
        {
            return 0.0f; // East (0°)
        }
        else
        {
            return (f32)M_PI; // West (180°)
        }
    }
    else
    {
        // Vertical movement is dominant.
        if (robot_t->dy > 0.0f)
        {
            return (f32)M_PI / 2.0f; // North (90°)
        }
        else
        {
            return (f32)-M_PI / 2.0f; // South (-90)
        }
    }

    //return std::atan2((robot_t->dx), (robot_t->dy));
}

/*
 * set the angle to between -pi and pi
 */

f32 normalize_angle(f32 theta)
{
    // while (theta > M_PI) theta -= M_TWOPI;
    // while (theta < -M_PI) theta += M_TWOPI;
    // return theta;
    return std::atan2(std::sin(theta), std::cos(theta));
}


// f32 abs_t(f32 value)
// {
//     if (value < 0.0f) {
//         return -value;
//     } else {
//         return value;
//     }

//}