#ifndef CONFIG_H
#define CONFIG_H
/* motors*/
#define MOTOR1_RPWM 5
#define MOTOR1_LPWM 18
#define MOTOR2_RPWM 16
#define MOTOR2_LPWM 17
#define MOTOR3_RPWM 18
#define MOTOR3_LPWM 19
#define MOTOR4_RPWM 20
#define MOTOR4_LPWM 21

/**************** Encoder Pins ****************/
#define ENCODER1_A  15
#define ENCODER1_B  14
#define ENCODER2_A  4
#define ENCODER2_B  3

/**************** Motion Parameters ****************/
#define WHEEL_DIAMETER_CM  6.6
 


/**************** RFID Parameters ****************/
#define RFID_SS_PIN  10     // SDA
#define RFID_RST_PIN 27    // RST
#define RFID_MOSI    23
#define RFID_MISO    19
#define RFID_SCK     21

// // IMU Parameters
// #define IMU_SDA  21
// #define IMU_SCL  22


#define WHEEL_RADIUS 1.0f
#define L 1.0f // distance between the wheels
#endif // CONFIG_H
