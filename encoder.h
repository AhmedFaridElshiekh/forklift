//
// Created by Ahmed Farid on 18/06/2025.
//

#ifndef ENCODER_H
#define ENCODER_H
#include "STD_TYPES.h"
#include <ESP32Encoder.h>
#define TICKS_PER_REVOLUTION 200 // (87*number of ticks per motor without gear revolution)

f32 get_right_distance();
f32 get_left_distance();
void encoders_init();


#endif //ENCODER_H
