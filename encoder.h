//
// Created by Ahmed Farid on 18/06/2025.
//

#ifndef ENCODER_H
#define ENCODER_H
#include "STD_TYPES.h"
#include <ESP32Encoder.h>
#define COUNTS_PER_REV 50
#define GEAR_RATIO 87.0
f32 get_right_distance();
f32 get_left_distance();
void encoders_init();


#endif  //ENCODER_H
