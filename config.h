#ifndef CONFIG_H
#define CONFIG_H

/**************** Motor Pins ****************/
#define MOTOR1_RPWM 5
#define MOTOR1_LPWM 18
#define MOTOR2_RPWM 16
#define MOTOR2_LPWM 17
#define MOTOR3_RPWM 25  
#define MOTOR3_LPWM 26  
#define MOTOR4_RPWM 32  
#define MOTOR4_LPWM 33  

/**************** Encoder Pins (PCNT Compatible) ****************/
#define ENCODER1_A 15
#define ENCODER1_B 14
#define ENCODER2_A 4
#define ENCODER2_B 2  // Changed from 3 (better for PCNT)

/**************** RFID Parameters (SPI) ****************/
#define RFID_SS_PIN 10   // CS/SDA pin
#define RFID_RST_PIN 27  // RST pin
#define RFID_MOSI 23     // SPI MOSI
#define RFID_MISO 19     // SPI MISO
#define RFID_SCK 18      // SPI SCK (changed from 21)

/**************** IMU Parameters (I2C) ****************/
#define IMU_SDA 21  // I2C Data
#define IMU_SCL 22  // I2C Clock

/**************** Motion Parameters ****************/
#define WHEEL_RADIUS 1.0f
#define L 1.0f  // distance between the wheels

#endif  // CONFIG_H