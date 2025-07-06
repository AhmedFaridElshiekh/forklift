#ifndef CONFIG_H 
#define CONFIG_H 
 
/**************** Motor Pins ****************/ 
#define MOTOR1_RPWM 14 
#define MOTOR1_LPWM 15 
#define MOTOR2_RPWM 12 
#define MOTOR2_LPWM 26 
// #define LIFT_RPWM 
// #define LIFT_LPWM 
 
#define rrpwm_motor_channel 0 
#define rlpwm_motor_channel 1 
#define lrpwm_motor_channel 2 
#define llpwm_motor_channel 3 
// #define lift_rpwm_motor_channel 4
// #define lift_lpwm_motor_channel 5
 
#define PWM_FREQ 5000 
#define PWM_RESOLUTION 8 
#define MAX_RPM 150.0f 
 
/**************** Encoder Pins (PCNT + INPUT_PULLUP) ****************/ 
#define ENCODER_right_A 19  // GPIO with internal pull-up capability for PCNT
#define ENCODER_right_B 18  // GPIO with internal pull-up capability for PCNT  
#define ENCODER_left_A 17  // GPIO with internal pull-up capability for PCNT
#define ENCODER_left_B 16  // GPIO with internal pull-up capability for PCNT
 
#define COUNTS_PER_REV 50 
#define GEAR_RATIO 87.0 
 
/**************** RFID Parameters (SPI) ****************/ 
#define RFID_SS_PIN   5 
#define RFID_RST_PIN  27 
#define RFID_MOSI     23 
#define RFID_MISO     19 
#define RFID_SCK      18 
 
/**************** IMU Parameters (I2C) ****************/ 
#define IMU_SDA 21 
#define IMU_SCL 22
#define INTERRUPT_PIN 4  
#define LED_PIN 2 
 
/**************** Motion Parameters ****************/ 
#define WHEEL_RADIUS .126f 
#define L .305f 
 
// /**************** Limit Switches (external pull-up/down resistors) ****************/ 
// #define LIMIT_SWITCH_UP 34  // Input-only GPIO (external pull-up/down required)
// #define LIMIT_SWITCH_DOWN 35  // Input-only GPIO (external pull-up/down required)
// #define LIMIT_SWITCH_FORK 36  // Input-only GPIO (external pull-up/down required)
 
#endif // CONFIG_H