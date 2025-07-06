# Forklift Robot Code Documentation

## File Structure

- forklift_extracted/
  - compute_velocity.cpp
  - compute_velocity.h
  - config.h
  - encoder.cpp
  - encoder.h
  - forklift_without_firebase.ino
  - fsm.cpp
  - fsm.h
  - imu.cpp
  - imu.h
  - kinematics.cpp
  - kinematics.h
  - motors.cpp
  - motors.h
  - pid.cpp
  - pid.h
  - readme.md
  - rfid.cpp
  - rfid.h
  - robot.h
  - state.h
  - STD_TYPES.h
  - types.h
  - UpdateOdometry.cpp
  - UpdateOdometry.h
  - angles.cpp
  - angles.h

---

## compute_velocity.cpp

### compute_velocity
- **Return Type:** `void`
- **Parameters:**
- `f32 set_point_angle`
- `f32 set_point_distance`
- `RobotData *robot_t`
- **Description:** No description provided.


## encoder.cpp

### get_right_distance
- **Return Type:** `f32`
- **Parameters:**
None
- **Description:** No description provided.

### get_left_distance
- **Return Type:** `f32`
- **Parameters:**
None
- **Description:** No description provided.

### encoders_clear
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.

### encoders_init
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.


## forklift_without_firebase.ino

### setup
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.

### loop
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.

### robot_init
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.


## fsm.cpp

### fsm_update
- **Return Type:** `void`
- **Parameters:**
- `RobotData *robot_t`
- `f32 points[][2]`
- `size_t num_points`
- **Description:** No description provided.

### set_target
- **Return Type:** `void`
- **Parameters:**
- `RobotData *robot_t`
- `f32 points[][2]`
- `size_t num_points`
- **Description:** No description provided.


## imu.cpp

### dmpDataReady
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.

### IMU_init
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.

### gyro_signals
- **Return Type:** `void`
- **Parameters:**
- `RobotData *robot_t`
- **Description:** No description provided.


## kinematics.cpp

### uni_to_diff
- **Return Type:** `void`
- **Parameters:**
- `f32 desired_angular_velocity`
- `f32 desired_linear_velocity`
- `RobotData *robot_t`
- **Description:** No description provided.


## motors.cpp

### init_motors
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.

### setMotor
- **Return Type:** `void`
- **Parameters:**
- `uint8_t rpwm_channel`
- `uint8_t lpwm_channel`
- `bool forward`
- `uint8_t speed`
- **Description:** No description provided.

### velocity_to_pwm
- **Return Type:** `u32`
- **Parameters:**
- `f32 velocity`
- `bool *direction`
- **Description:** No description provided.

### motors
- **Return Type:** `void`
- **Parameters:**
- `RobotData *robot_t`
- **Description:** No description provided.

### stop_motors
- **Return Type:** `void`
- **Parameters:**
- `RobotData *robot_t`
- **Description:** No description provided.

### fork
- **Return Type:** `void`
- **Parameters:**
- `RobotData *robot_t`
- **Description:** No description provided.


## pid.cpp

### PIDController_Init
- **Return Type:** `void`
- **Parameters:**
- `PIDController *pid`
- **Description:** No description provided.

### PIDController_Update
- **Return Type:** `f32`
- **Parameters:**
- `PIDController *pid`
- `f32 setpoint`
- `f32 measurement`
- **Description:** No description provided.


## rfid.cpp

### initRFID
- **Return Type:** `void`
- **Parameters:**
None
- **Description:** No description provided.

### readXYFromCard
- **Return Type:** `bool`
- **Parameters:**
- `byte &xValue`
- `byte &yValue`
- **Description:** No description provided.


## UpdateOdometry.cpp

### update_odometry
- **Return Type:** `void`
- **Parameters:**
- `RobotData *robot_t`
- **Description:** No description provided.


## angles.cpp

### has_reached_target
- **Return Type:** `u8`
- **Parameters:**
- `RobotData *robot_t`
- **Description:** No description provided.

### compute_desired_angle
- **Return Type:** `f32`
- **Parameters:**
- `RobotData *robot_t`
- **Description:** No description provided.

### normalize_angle
- **Return Type:** `f32`
- **Parameters:**
- `f32 theta`
- **Description:** No description provided.
