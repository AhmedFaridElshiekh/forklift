//
// Created by Ahmed Farid on 18/06/2025.
//

#include "encoder.h"
#include "angles.h"
#include "kinematics.h"
int32_t last_ticks_left = 0;
int32_t last_ticks_right = 0;
ESP32Encoder encoder_right;
ESP32Encoder encoder_left;
f32 get_right_distance() {
  int32_t right_ticks_new = encoder_right.getCount();
  int32_t delta_ticks_right = right_ticks_new - last_ticks_right;
  last_ticks_right = right_ticks_new;
  f32 distance_right = 2 * M_PI * WHEEL_RADIUS * delta_ticks_right / (TICKS_PER_REVOLUTION);

  return distance_right;
}

f32 get_left_distance() {
  int32_t left_ticks_new = encoder_left.getCount();
  int32_t delta_ticks_left = left_ticks_new - last_ticks_left;
  last_ticks_left = left_ticks_new;
  f32 distance_left = 2 * M_PI * WHEEL_RADIUS * delta_ticks_left / (TICKS_PER_REVOLUTION);
  return distance_left;
}
void encoders_init() {

  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  // use pin 19 and 18 for the first encoder
  encoder_right.attachFullQuad(19, 18);
  // use pin 17 and 16 for the second encoder
  encoder_left.attachFullQuad(17, 16);

  // clear the encoder's raw count and set the tracked count to zero
  encoder_right.clearCount();
  encoder_left.clearCount();
  // these are inplace but thinking for a use for it
  // encoder2.pauseCount();
  // encoder2.resumeCount();
}
