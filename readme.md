# Robot Navigation System Documentation

## File Hierarchy

```
Robot Navigation System/
├── Core Headers/
│   ├── STD_TYPES.h          # Standard type definitions
│   ├── robot.h              # Main robot data structure
│   └── state.h              # Robot state enumeration
├── Control System/
│   ├── fsm.h/cpp            # Finite State Machine
│   ├── pid.h/cpp            # PID Controller implementation
│   └── compute_velocity.h/cpp # Velocity computation
├── Motion & Kinematics/
│   ├── kinematics.h/cpp     # Unicycle to differential drive conversion
│   ├── angles.h/cpp         # Angle calculations and normalization
│   └── UpdateOdometry.h/cpp # Odometry and position tracking
├── Hardware Interface/
│   ├── motors.h/cpp         # Motor control and PWM conversion
│   └── encoder.h/cpp        # Encoder distance measurement
└── main.cpp                 # Main program entry point
```

## File Documentation

### Core Headers

#### STD_TYPES.h
**Purpose**: Defines standard data types used throughout the system.

**Type Definitions**:
- `u8`, `u16`, `u32`, `u64` - Unsigned integer types (8, 16, 32, 64 bits)
- `s8`, `s16`, `s32`, `s64` - Signed integer types (8, 16, 32, 64 bits)
- `f32`, `f64` - Floating point types (32, 64 bits)

#### state.h
**Purpose**: Defines the robot's operational states.

**Enumerations**:
```cpp
typedef enum {
    IDLE,              // Robot is idle, waiting for commands
    ROTATE_TO_TARGET,  // Robot is rotating to face target direction
    MOVE_FORWARD,      // Robot is moving forward to target
    ARRIVED,           // Robot has reached current waypoint
    CHARGING,          // Robot is in charging mode
    LIFT               // Robot is performing lift operation
} RobotState;
```

#### robot.h
**Purpose**: Main robot data structure containing all robot parameters and state.

**Structures**:
```cpp
typedef enum {
    BATTERY_NORMAL,    // Battery level >50%
    BATTERY_LOW,       // Battery level 20-50%
    BATTERY_CRITICAL   // Battery level <20%
} BatteryState;

typedef struct {
    // Position data
    f32 x_current, y_current;     // Current robot position (m)
    f32 x_target, y_target;       // Target position (m)
    f32 dx, dy;                   // Position error (m)
    f32 r_speed, l_speed;         // Right/left wheel speeds (m/s)
    u32 r_speed_pwm, l_speed_pwm; // PWM values for motors
    
    // Sensor data
    f32 gyro[3];                  // Gyroscope readings [x,y,z] (rad/s)
    f32 accel[3];                 // Accelerometer readings [x,y,z] (m/s²)
    f32 distance_encoders;        // Accumulated distance from encoders (m)
    
    // System state
    RobotState state;             // Current robot state
    BatteryState battery_state;   // Battery status
    u8 ID;                        // Robot identifier
    
    // Control data
    PIDController pid_linear;     // PID controller for linear velocity
    PIDController pid_heading;    // PID controller for heading
} RobotData;
```

### Control System

#### fsm.h/cpp
**Purpose**: Implements the finite state machine that controls robot behavior.

**Constants**:
- Static variable `current_waypoint` - Tracks current waypoint index

**Functions**:

```cpp
void fsm_update(RobotData* robot_t, f32 points[][2], size_t num_points);
```
- **Description**: Main state machine update function that processes robot states
- **Parameters**: 
  - `robot_t`: Pointer to robot data structure
  - `points`: Array of waypoint coordinates [x,y]
  - `num_points`: Number of waypoints
- **Functionality**: Handles state transitions and calls appropriate control functions based on current state

```cpp
void set_target(RobotData* robot_t, f32 points[][2], size_t num_points);
```
- **Description**: Sets the next target waypoint for the robot
- **Parameters**: Same as fsm_update
- **Functionality**: Updates target coordinates and transitions to ROTATE_TO_TARGET state

#### pid.h/cpp
**Purpose**: Implements PID (Proportional-Integral-Derivative) controller for robot motion control.

**Constants**:
- `MAX_ANGULAR_VELOCITY`: 10.0f rad/s - Maximum angular velocity
- `MAX_LINEAR_VELOCITY`: 10.0f m/s - Maximum linear velocity
- `KP_LINEAR_VELOCITY`: 1.0f - Proportional gain for linear velocity
- `KI_LINEAR_VELOCITY`: 0.0f - Integral gain for linear velocity
- `KD_LINEAR_VELOCITY`: 0.0f - Derivative gain for linear velocity
- `KP_HEADING`: 1.0f - Proportional gain for heading control
- `KI_HEADING`: 1.0f - Integral gain for heading control
- `KD_HEADING`: 1.0f - Derivative gain for heading control
- `TAU`: 0.5f - Derivative filter time constant
- `DT`: 0.0001f - Sampling time (seconds)
- `DESIRED_DISTANCE`: 1.0f - Target distance for linear control
- `HEADING_C`: 1 - Flag for heading control
- `DISTANCE_C`: 0 - Flag for distance control

**Structures**:
```cpp
typedef struct {
    f32 Kp, Ki, Kd;              // PID gains
    f32 tau;                     // Derivative filter time constant
    f32 limMin, limMax;          // Output limits
    f32 limMinInt, limMaxInt;    // Integrator limits
    f32 T;                       // Sample time
    f32 integrator;              // Integrator state
    f32 prevError;               // Previous error for integrator
    f32 differentiator;          // Differentiator state
    f32 prevMeasurement;         // Previous measurement for differentiator
    f32 out;                     // Controller output
    u8 flag;                     // Controller type flag (0=distance, 1=angle)
} PIDController;
```

**Functions**:

```cpp
void PIDController_Init(PIDController* pid);
```
- **Description**: Initializes PID controller variables to zero
- **Parameters**: `pid` - Pointer to PID controller structure

```cpp
f32 PIDController_Update(PIDController* pid, f32 setpoint, f32 measurement);
```
- **Description**: Updates PID controller and returns control output
- **Parameters**: 
  - `pid`: Pointer to PID controller
  - `setpoint`: Desired value
  - `measurement`: Current measured value
- **Returns**: Control output value
- **Features**: Includes anti-windup, derivative filtering, and angle normalization for heading control

#### compute_velocity.h/cpp
**Purpose**: Computes desired linear and angular velocities based on robot state.

**Functions**:

```cpp
void compute_velocity(f32 set_point_angle, f32 set_point_distance, RobotData* robot_t);
```
- **Description**: Computes velocities using PID controllers and converts to differential drive
- **Parameters**:
  - `set_point_angle`: Desired heading angle (radians)
  - `set_point_distance`: Desired distance to travel (meters)
  - `robot_t`: Pointer to robot data structure
- **Functionality**: 
  - In MOVE_FORWARD state: Computes both linear and angular velocities
  - In ROTATE_TO_TARGET state: Only computes angular velocity (linear = 0)

### Motion & Kinematics

#### angles.h/cpp
**Purpose**: Handles angle calculations, normalization, and target detection.

**Constants**:
- `TOLERANCEANGLE`: 0.0523598776f (≈3°) - Angular tolerance in radians
- `M_PI`: 3.14159265358979323846f - Pi constant
- `M_TWOPI`: 6.28318530717958647692f - 2*Pi constant

**Functions**:

```cpp
u8 has_reached_target(RobotData* robot_t);
```
- **Description**: Checks if robot has reached target position
- **Parameters**: `robot_t` - Pointer to robot data
- **Returns**: 1 if reached (both dx and dy < 0.01), 0 otherwise

```cpp
f32 compute_desired_angle(RobotData* robot_t);
```
- **Description**: Computes the desired heading angle to reach target
- **Parameters**: `robot_t` - Pointer to robot data
- **Returns**: Desired angle in radians (0°=East, 90°=North, 180°=West, -90°=South)
- **Logic**: Uses dominant movement component (larger of |dx| or |dy|) to determine direction

```cpp
f32 normalize_angle(f32 theta);
```
- **Description**: Normalizes angle to range [-π, π]
- **Parameters**: `theta` - Input angle in radians
- **Returns**: Normalized angle
- **Method**: Uses `atan2(sin(theta), cos(theta))` for robust normalization

#### kinematics.h/cpp
**Purpose**: Converts between unicycle and differential drive kinematics.

**Constants**:
- `WHEEL_RADIUS`: 1.0f - Wheel radius (meters)
- `L`: 1.0f - Distance between wheels (meters)

**Functions**:

```cpp
void uni_to_diff(f32 desired_angular_velocity, f32 desired_linear_velocity, RobotData* robot_t);
```
- **Description**: Converts unicycle model velocities to differential drive wheel speeds
- **Parameters**:
  - `desired_angular_velocity`: Desired angular velocity (rad/s)
  - `desired_linear_velocity`: Desired linear velocity (m/s)
  - `robot_t`: Pointer to robot data structure
- **Formulas**:
  - Right wheel: `(2*v + ω*L) / (2*R)`
  - Left wheel: `(2*v - ω*L) / (2*R)`
  - Where v=linear velocity, ω=angular velocity, L=wheelbase, R=wheel radius

#### UpdateOdometry.h/cpp
**Purpose**: Updates robot position and orientation using sensor fusion.

**Global Variables**:
- `x_RFID`, `y_RFID`: RFID-based position corrections
- `theta_encoders`: Heading from encoder integration
- `theta`: Current heading from gyroscope

**Functions**:

```cpp
void update_odometry(RobotData* robot_t);
```
- **Description**: Updates robot's position estimate using encoders and sensors
- **Parameters**: `robot_t` - Pointer to robot data structure
- **Process**:
  1. Calculates center distance from wheel encoders
  2. Updates position using: `x += d*cos(θ)`, `y += d*sin(θ)`
  3. Updates heading from wheel difference: `θ += (Dr - Dl)/L`
  4. Updates position errors: `dx = x_target - x_current`
  5. Includes provisions for RFID-based position corrections

### Hardware Interface

#### motors.h/cpp
**Purpose**: Interfaces with motor hardware, converting velocities to PWM signals.

**Constants**:
- `MAX_RPM`: 150.0f - Maximum motor RPM
- `PWM_MAX`: 255 - Maximum PWM value (8-bit)
- `FORWARD`: 1 - Forward direction flag
- `BACKWARD`: 0 - Backward direction flag

**Functions**:

```cpp
u32 velocity_to_pwm(f32 velocity, bool* direction);
```
- **Description**: Converts velocity to PWM value and direction
- **Parameters**:
  - `velocity`: Input velocity (can be negative)
  - `direction`: Pointer to store direction flag
- **Returns**: PWM value (0-255)
- **Logic**: Sets direction based on sign, returns absolute value clamped to PWM_MAX

```cpp
void motors(RobotData* robot_t);
```
- **Description**: Applies computed velocities to motors
- **Parameters**: `robot_t` - Pointer to robot data structure
- **Functionality**: 
  - Converts left/right speeds to PWM values
  - Sets motor directions
  - Includes debug output for speed monitoring
  - Contains commented hardware interface calls

```cpp
void stop_motors(RobotData* robot_t);
```
- **Description**: Stops all motors and resets speed values
- **Parameters**: `robot_t` - Pointer to robot data structure

#### encoder.h/cpp
**Purpose**: Interfaces with wheel encoders to measure distance traveled.

**Functions**:

```cpp
f32 get_right_distance();
f32 get_left_distance();
```
- **Description**: Get distance traveled by right/left wheels since last call
- **Returns**: Distance in meters
- **Status**: Currently return 0 (placeholder implementation)
- **Intended Logic**: 
  - Calculate tick differences since last read
  - Convert to distance: `D = 2πr * ticks / ticks_per_revolution`

### Main Program

#### main.cpp
**Purpose**: Main program entry point and robot initialization.

**Global Variables**:
- `robot_t`: Main robot data structure instance

**Functions**:

```cpp
int main(void);
```
- **Description**: Main program loop
- **Functionality**:
  1. Initializes robot system
  2. Defines waypoint array: (0,1) → (1,1) → (1,0) → (2,0)
  3. Sets initial target
  4. Runs infinite control loop calling `fsm_update()`

```cpp
void robot_init();
```
- **Description**: Initializes all robot parameters and subsystems
- **Functionality**:
  - Zeros position and sensor data
  - Configures PID controller parameters for both linear and heading control
  - Sets initial state to IDLE
  - Sets battery state to NORMAL
  - Initializes PID controllers

## System Operation Flow

1. **Initialization**: Robot starts in IDLE state with initialized PID controllers
2. **Target Setting**: First waypoint is set, state changes to ROTATE_TO_TARGET
3. **Rotation Phase**: Robot rotates until facing target (within tolerance)
4. **Movement Phase**: State changes to MOVE_FORWARD, robot moves toward target
5. **Arrival Check**: When target is reached, state changes to ARRIVED
6. **Waypoint Progression**: Next waypoint is set, cycle repeats
7. **Task Completion**: When final waypoint is reached, state changes to LIFT

## Key Features

- **Finite State Machine**: Robust state-based control system
- **PID Control**: Separate controllers for linear and angular motion
- **Sensor Fusion**: Combines encoder and gyroscope data for position estimation
- **Safety Features**: Battery monitoring and motor limiting
- **Modular Design**: Clean separation of concerns across multiple files
- **Hardware Abstraction**: Clear interface between control logic and hardware

## Configuration Parameters

The system can be tuned by modifying constants in the respective header files:
- PID gains in `pid.h`
- Physical parameters in `kinematics.h`
- Tolerances in `angles.h`
- Motor limits in `motors.h`