#include "STD_TYPES.h"
// #include "encoder.h"
// #include "fsm.h"
#include "imu.h"
// #include "pid.h"
#include "robot.h"

// #include "rfid.h"
// #include "motors.h"

RobotData robot_t;
void robot_init();

// Waypoints (x, y) in fixed-point (e.g., 0.1m resolution)
f32 waypoints[][2] = {{1, 0}, {2, 0}, {2, 1}

};
size_t num_waypoints = sizeof(waypoints) / sizeof(waypoints[0]);

void setup() {
  Serial.begin(115200);

  // // put your setup code here, to run once:
  // robot_init();
  // encoders_init();
  // // initRFID();
  // init_motors();

  // this is from mpu library
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  //  	Serial.begin(115200);
  IMU_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  // fsm_update(&robot_t, waypoints, num_waypoints);
  gyro_signals(&robot_t);
}
/*
 * Initializes the robot's data structure and sets up the PID controllers.
 */
void robot_init() {
  // Initialize robot_t data structure
  robot_t.x_current = 0.0f;
  robot_t.y_current = 0.0f;
  robot_t.x_target = 0.0f;
  robot_t.y_target = 0.0f;
  robot_t.dx = 0.0f;
  robot_t.dy = 0.0f;

  // Clear sensor data
  for (int i = 0; i < 3; i++) {
    robot_t.accel[i] = 0.0f;
    robot_t.gyro[i] = 0.0f;
  }
  robot_t.distance_encoders = 0.0f;

  // Initialize PID controllers
  PIDController_Init(&robot_t.pid_heading);

  // Set PID parameters (previously globals)

  robot_t.pid_heading.Kp = KP_HEADING;
  robot_t.pid_heading.Ki = KI_HEADING;
  robot_t.pid_heading.Kd = KD_HEADING;
  robot_t.pid_heading.tau = TAU;
  robot_t.pid_heading.limMin = -MAX_ANGULAR_VELOCITY;
  robot_t.pid_heading.limMax = MAX_ANGULAR_VELOCITY;
  robot_t.pid_heading.limMinInt = PID_LIM_MIN_INT;
  robot_t.pid_heading.limMaxInt = PID_LIM_MAX_INT;
  robot_t.pid_heading.T = DT;
  robot_t.pid_heading.flag = HEADING_C;

  // Initialize speeds
  robot_t.r_speed = 0.0f;
  robot_t.l_speed = 0.0f;
  robot_t.r_speed_pwm = 0;
  robot_t.l_speed_pwm = 0;
  // Initial state
  robot_t.state = IDLE;
  robot_t.battery_state = BATTERY_NORMAL;
    robot_t.fork_state=NORMAL;
      robot_t.ID=1;



}
