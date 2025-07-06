//
// Created by Ahmed Farid on 18/06/2025.
//

#ifndef ENCODER_H
#define ENCODER_H
#include "STD_TYPES.h"
#include <ESP32Encoder.h>

f32 get_right_distance();
f32 get_left_distance();
void encoders_init();
void encoders_clear();

#endif // ENCODER_H
