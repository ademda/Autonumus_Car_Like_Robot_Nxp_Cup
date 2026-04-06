//THIS CODE IS FOR THE TUNING OF THE ROBOT: THE COMMANDS WILL BE SENT FROM THE PYTHON INTERFACE 
//THE OTHER BRANCH: ADEM_BRANCH HAS THE CODE THAT IS THE CODE FOR THE COMPETITION 
//THIS CODE CONTAINS : ASSERVISSEMENT PAR ROUE
#include <Arduino.h>
#include <Servo.h>
#include <TimerOne.h>
#include <QuadEncoder.h>
#include "vision.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ToFFilter.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_SSD1306.h>
#include "strategy.h"
#include "infrared.h"
//#define DEBUG 1

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

#define UART_TX 1
#define UART_RX 0

#define LEFT_ENC_CH1 3
#define LEFT_ENC_CH2 2

#define RIGHT_ENC_CH1 5
#define RIGHT_ENC_CH2 4

#define RIGHTMOTOR_FWD_PWM 23
#define RIGHTMOTOR_BWD_PWM 22

#define LEFTMOTOR_FWD_PWM 6
#define LEFTMOTOR_BWD_PWM 7

#define TIM1_PIN 8
#define TIM3_PIN 9

#define SERVO_PIN 17  
#define JACK_PIN 9

#define SPI_CS_PIN 10
#define SPI_MOSI_PIN 11  
#define SPI_MISO_PIN 12
#define SPI_SCLK_PIN 13

#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

#define TOF_ADDR_1 0x30

/***************** CONTROLLER DEFINES ****************/
#define RIGHT_VEL_KP 0.25
#define RIGHT_VEL_KI 0.007
#define RIGHT_VEL_KD 0.0

#define LEFT_VEL_KP 0.025
#define LEFT_VEL_KI 0.007
#define LEFT_VEL_KD 0.0

#define STEERING_KP 1.0
#define STEERING_KI 0.0
#define STEERING_KD 0.0

#define MAX_SERVO_ANGLE 135
#define MIN_SERVO_ANGLE 42
#define MAX_MOTOR_CMD 255
#define MIN_MOTOR_CMD 0
#define MAX_STEERING_ERROR_SUM 120
#define MAX_VEL_ERROR_SUM 35000

#define WHEEL_GAIN  1.000
#define CONTROL_LOOP_DT_MS 5
#define VELOCITY_CALC_DT_MS 5
#define STOP_DISTANCE 400


// ── Trapezoidal velocity profile ──────────────────────────────────
#define ACCEL_RATE_MM_S2        24000.0f  // 16000.0f     // mm/s² ramp up rate
#define DECEL_RATE_MM_S2        24000.0f  // mm/s² ramp down rate (faster = tighter braking)
#define SPEED_MAX_MM_S          2000.0f  // top speed on a straight
#define SPEED_MIN_TURN_MM_S     1400.0f // minimum speed in the sharpest turn
#define SPEED_ANGLE_DEADBAND    20.0f     // degrees of angle error treated as "straight"

/**************** ODOMETRY DEFINES ****************/
#define LEFT_ENCODER_CPR 408
#define RIGHT_ENCODER_CPR 408
#define LEFT_WHEEL_DIAMETER_MM 67.58
#define RIGHT_WHEEL_DIAMETER_MM 67.58
#define WHEEL_BASE_MM 194
#define SERVO_INIT_ANGLE 87

float K_STRAIGHT = 2.5;
float K_SHARP = 6;
float GAIN_THRESHOLD = 33.0;
float CAMERA_SMOOTHING = 0.7;

float filtered_camera_angle = 87.0;

/********************** ODOMETRY VARIABLES *****************/
volatile double left_wheel_curr_vel_mm_s, right_wheel_curr_vel_mm_s, robot_curr_vel_mm_s;
volatile double prev_left_wheel_dist_mm, prev_right_wheel_dist_mm, prev_robot_dist_mm;
volatile double left_wheel_distance_mm, right_wheel_distance_mm, robot_distance_mm;
volatile double curr_orientation_deg;
volatile double left_vel_filtered, right_vel_filtered;

volatile double left_wheel_dist_prev_vel_calc = 0;
volatile double right_wheel_dist_prev_vel_calc = 0;
volatile uint32_t last_vel_calc_ms = 0;

/************************ PID VARIABLES *********************/
volatile float right_vel_kp = RIGHT_VEL_KP, right_vel_ki = RIGHT_VEL_KI, right_vel_kd = RIGHT_VEL_KD;
volatile float left_vel_kp = LEFT_VEL_KP, left_vel_ki = LEFT_VEL_KI, left_vel_kd = LEFT_VEL_KD;
volatile float steering_kp = STEERING_KP, steering_ki = STEERING_KI, steering_kd = STEERING_KD;
volatile uint8_t K_heading = 2;

/**************** ACTUATORS VARIABLES ********/
volatile int32_t right_motor_cmd, left_motor_cmd;
volatile int16_t servo_angle_cmd_deg = 93;

/**************** SENSORS VARIABLES ***********/
volatile int32_t left_ticks_i32, right_ticks_i32;

/**************** CONTROL VARIABLES *****/
// VELOCITY CONTROL
volatile double left_motor_vel_setpoint_mm_s, right_motor_vel_setpoint_mm_s, robot_vel_setpoint_mm_s;
volatile double prev_left_motor_vel_setpoint_mm_s, prev_right_motor_vel_setpoint_mm_s;
volatile double left_motor_vel_error_mm_s, right_motor_vel_error_mm_s, robot_vel_error_mm_s;
volatile double left_motor_vel_pid_output, right_motor_vel_pid_output;
volatile double left_motor_vel_error_sum_mm_s, right_motor_vel_error_sum_mm_s;
volatile double left_motor_vel_last_error_mm_s, right_motor_vel_last_error_mm_s;

// STEERING CONTROL
volatile float orientation_setpoint_deg, orientation_error_deg, orientation_error_sum_deg;
volatile float orientation_last_error_deg;
volatile float servo_angle_pid_output;

// DISTANCE CONTROL
volatile float distance_setpoint_mm, prev_robot_distance_mm;
volatile float distance_error_mm;
volatile bool distance_control_enable = false;
volatile bool distance_reached = false;

// EMERGENCY STOP
volatile bool emergency_stop_enable = false;

// TRAPEZOIDAL VELOCITY PROFILE
volatile float left_vel_ramped_mm_s  = 0.0f;
volatile float right_vel_ramped_mm_s = 0.0f;
volatile float left_vel_target_mm_s  = 0.0f;
volatile float right_vel_target_mm_s = 0.0f;
volatile uint32_t total_time_at_max_speed_ms = 0;  // cumulative time at max speed
volatile uint32_t total_time_at_min_speed_ms = 0;  // cumulative time at min speed
volatile bool max_speed_reached = false;           // did robot reach max speed
volatile bool min_speed_reached = false;           // did robot reach min speed

// CAMERA
volatile float last_camera_angle;
volatile float last_camera_angle_updated;
volatile uint32_t servo_wait = 0;

VL53L0X_RangingMeasurementData_t tof_measure;
float left_tof_distance, center_tof_distance, right_tof_distance;
uint32_t last_tof_test = 0;
const uint32_t TOF_INTERVAL_MS = 100;
bool cube_detected = false;
ToFFilter tofFilter;

/********* INSTANCES ******/
QuadEncoder left_encoder(1, LEFT_ENC_CH1, LEFT_ENC_CH2);
QuadEncoder right_encoder(2, RIGHT_ENC_CH1, RIGHT_ENC_CH2);
Servo steer_servo;
Vision vision;

/*********** DEBUG VARIABLES ****/
uint32_t last_debug = 0;
uint32_t start_time = 0;
extern uint32_t read_ir_start_time;

/* ToF */
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();

/********* SSD1306 OLED DISPLAY ******/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/************** FUNCTIONS DECLARATIONS *****/
void ReadEncoders();
void RotateMotors();
void SetServoAngle();
void ConvertTicksToDistance();
void ConvertDistanceToVel();
void CalculateVelError();
void CalculateVelPID();
void CalculateOrientationError();
void CalculateSteeringPID();
void VelControllerRoutine();
void VelOdomRoutine();
void CalculateDistanceError();
void StopMotors();
void ComputeAngleBasedSpeed();

void EmptyFunction(){
}

void NavRoutine(){
  VelOdomRoutine();
  ComputeAngleBasedSpeed();   // angle-based speed + trapezoidal ramp
  VelControllerRoutine();

  if (cube_detected == true){
    StopMotors();
  } else {
    if (ir_stop_triggered == true){
      // IR override: clamp to a safe fixed speed during stop zone
      left_motor_vel_setpoint_mm_s  = 750;
      right_motor_vel_setpoint_mm_s = 750;
    }
    RotateMotors();
  }

  if (!ir_stop_triggered && millis() - start_time > read_ir_start_time){
    ReadIRSensors();
  }

  if (millis() - servo_wait >= 750 && cube_detected == false){
    SetServoAngle();
  }
}

void VelOdomRoutine(){
  ReadEncoders();
  ConvertTicksToDistance();
  ConvertDistanceToVel();
}

void VelControllerRoutine(){
  CalculateVelError();
  CalculateVelPID();
}

void softwareReset(){
  SCB_AIRCR = 0x05FA0004;
}

void readToFsNonBlocking() {
  if (millis() - last_tof_test >= TOF_INTERVAL_MS) {
    uint16_t distance;

    if (tof1.isRangeComplete()) {
      distance = tof1.readRange();
      if (!tof1.timeoutOccurred()) {
        center_tof_distance = tofFilter.filter(distance) * 1000;
      }
    }

    if (center_tof_distance < STOP_DISTANCE){
      cube_detected = true;
    } else if (center_tof_distance > STOP_DISTANCE){
      cube_detected = false;
    }

    last_tof_test = millis();

    #ifdef DEBUG
    Serial.print("left_tof_distance: "); Serial.print(left_tof_distance); Serial.print(" mm | ");
    Serial.print("center_tof_distance: "); Serial.print(center_tof_distance); Serial.print(" mm | ");
    Serial.print("right_tof_distance: "); Serial.print(right_tof_distance); Serial.println(" mm | ");
    #endif
  }
}

/**
 * ComputeAngleBasedSpeed()
 *
 * If velocity_profile_enabled (strategy 3):
 *   Maps the current filtered camera steering error to a longitudinal
 *   speed target, then steps the actual setpoint toward it using a
 *   trapezoidal ramp (separate accel and decel rates).
 *
 * If velocity_profile_enabled is false (strategies 0-2):
 *   Uses constant velocity from strategy_speed (no acceleration profile).
 *
 * Called once per Timer1 tick (every CONTROL_LOOP_DT_MS ms).
 */
void ComputeAngleBasedSpeed() {
  if (ir_stop_triggered == false){
    const float dt_s = CONTROL_LOOP_DT_MS / 1000.0f;

    if (velocity_profile_enabled) {
      // ── STRATEGY 3: VELOCITY PROFILE ENABLED ────────────────────

      // ── 1. Angle error from straight (87°) ──────────────────────────
      float angle_error_deg = abs(filtered_camera_angle - 87.0f);

      // Deadband: ignore small noise so speed is stable on straights
      // if (angle_error_deg < SPEED_ANGLE_DEADBAND) {
      //   angle_error_deg = 0.0f;
      // }

      // ── 2. Map angle error → desired speed ──────────────────────────
      // Below GAIN_THRESHOLD → full speed.
      // Above GAIN_THRESHOLD → linearly interpolate down to min speed.
      float desired_speed;
      if (angle_error_deg <= GAIN_THRESHOLD) {
        desired_speed = VELOCITY_PROFILE_MAX_SPEED;
      } else {
        // float t = (angle_error_deg - GAIN_THRESHOLD) / (90.0f - GAIN_THRESHOLD);
        // t = constrain(t, 0.0f, 1.0f);
        // desired_speed = VELOCITY_PROFILE_MAX_SPEED - t * (VELOCITY_PROFILE_MAX_SPEED - VELOCITY_PROFILE_MIN_SPEED);
        desired_speed = VELOCITY_PROFILE_MIN_SPEED;
      }

      left_vel_target_mm_s  = desired_speed;
      right_vel_target_mm_s = desired_speed;

      // ── 3. Trapezoidal ramp — left wheel ────────────────────────────
      float left_error = left_vel_target_mm_s - left_vel_ramped_mm_s;
      if (left_error > 0.0f) {
        left_vel_ramped_mm_s += ACCEL_RATE_MM_S2 * dt_s;
        if (left_vel_ramped_mm_s > left_vel_target_mm_s)
          left_vel_ramped_mm_s = left_vel_target_mm_s;
      } else if (left_error < 0.0f) {
        left_vel_ramped_mm_s -= DECEL_RATE_MM_S2 * dt_s;
        if (left_vel_ramped_mm_s < left_vel_target_mm_s)
          left_vel_ramped_mm_s = left_vel_target_mm_s;
      }

      // ── 4. Trapezoidal ramp — right wheel ───────────────────────────
      float right_error = right_vel_target_mm_s - right_vel_ramped_mm_s;
      if (right_error > 0.0f) {
        right_vel_ramped_mm_s += ACCEL_RATE_MM_S2 * dt_s;
        if (right_vel_ramped_mm_s > right_vel_target_mm_s)
          right_vel_ramped_mm_s = right_vel_target_mm_s;
      } else if (right_error < 0.0f) {
        right_vel_ramped_mm_s -= DECEL_RATE_MM_S2 * dt_s;
        if (right_vel_ramped_mm_s < right_vel_target_mm_s)
          right_vel_ramped_mm_s = right_vel_target_mm_s;
      }

      // ── 5. Track cumulative time at max speed ─────────────────────────
      // Check if current velocity is close to max speed (within 5%)
      if (left_vel_ramped_mm_s >= VELOCITY_PROFILE_MAX_SPEED * 0.95f) {
        total_time_at_max_speed_ms += CONTROL_LOOP_DT_MS;
        max_speed_reached = true;
      }
      
      // ── 6. Track cumulative time at min speed ─────────────────────────
      // Check if current velocity is close to min speed (within 5%)
      if (left_vel_ramped_mm_s <= VELOCITY_PROFILE_MIN_SPEED * 1.05f && left_vel_ramped_mm_s > 0) {
        total_time_at_min_speed_ms += CONTROL_LOOP_DT_MS;
        min_speed_reached = true;
      }
      
      // ── 7. Feed ramped values into PID setpoints ────────────────────
      left_motor_vel_setpoint_mm_s  = left_vel_ramped_mm_s;
      right_motor_vel_setpoint_mm_s = right_vel_ramped_mm_s;

    } else {
      // ── STRATEGIES 0-2: CONSTANT VELOCITY (NO PROFILE) ──────────────

      // Use constant velocity from strategy_speed
      left_vel_target_mm_s  = strategy_speed;
      right_vel_target_mm_s = strategy_speed;
      
      // Pass setpoints directly without ramping
      left_motor_vel_setpoint_mm_s  = strategy_speed;
      right_motor_vel_setpoint_mm_s = strategy_speed;
    }
  }  
}

void setup() {
  pinMode(JACK_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  vision.begin();
  delay(3000);
  vision.pixy.setLamp(1, 1);

  pinMode(IR_1_PIN, INPUT);
  pinMode(IR_2_PIN, INPUT);
  pinMode(IR_3_PIN, INPUT);
  pinMode(IR_4_PIN, INPUT);

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(F("HELLO"));
  display.display();

  CalibrateIRSensors();

  left_encoder.setInitConfig();
  left_encoder.init();
  right_encoder.setInitConfig();
  right_encoder.init();
  left_encoder.write(0);
  right_encoder.write(0);
  servo_wait = millis();

  steer_servo.attach(SERVO_PIN);
  steer_servo.write(SERVO_INIT_ANGLE);
  Serial1.begin(115200);

  Wire.begin();
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.setClock(400000);

  if (!tof1.begin(0x29, &Wire)) {
    Serial.println("Failed to boot ToF 1");
  }
  tof1.setAddress(TOF_ADDR_1);
  tof1.startRangeContinuous(50);

  tofFilter.setOffset(15);
  tofFilter.setRangeLimits(20, 20000);
  tofFilter.setPublishInterval(1000 / TOF_INTERVAL_MS);

  uint32_t debounce_time = millis();
  Strategy_Init(display);
  while(digitalRead(JACK_PIN) == LOW) {
    if(millis() - debounce_time > 50) {
      Strategy_Poll(display);
      delay(10);
    }
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20);
  display.println(F("JACK TRIGGERED"));
  display.display();
  delay(100);
  delay(2000);
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  // Initialise ramp state — robot starts from rest and accelerates smoothly
  left_vel_ramped_mm_s  = 0.0f;
  right_vel_ramped_mm_s = 0.0f;
  left_motor_vel_setpoint_mm_s  = 0.0f;
  right_motor_vel_setpoint_mm_s = 0.0f;

  Timer1.initialize(CONTROL_LOOP_DT_MS * 1000);
  Timer1.attachInterrupt(NavRoutine);

  start_time = millis();
}

void loop() {
  uint32_t current_time = millis();
  uint32_t time_diff = current_time - start_time;

  if (time_diff > 3000) {
    if (ir_stop_triggered == true){
      readToFsNonBlocking();
      
      // Display speed tracking results when robot stops
      display.ssd1306_command(SSD1306_DISPLAYON);
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      
      display.setCursor(0, 0);
      display.print(F("Max Speed Time: "));
      display.print(total_time_at_max_speed_ms);
      display.println(F(" ms"));
      
      display.setCursor(0, 10);
      display.print(F("Min Speed Time: "));
      display.print(total_time_at_min_speed_ms);
      display.println(F(" ms"));
      
      display.setCursor(0, 20);
      display.print(F("Max Reached: "));
      display.println(max_speed_reached ? F("YES") : F("NO"));
      
      display.setCursor(0, 30);
      display.print(F("Min Reached: "));
      display.println(min_speed_reached ? F("YES") : F("NO"));
      
      display.display();
    }
  }

  String mode;
  float distance;
  last_camera_angle = vision.calculate_steering_angle(mode, distance);
  last_camera_angle = 180 - last_camera_angle;
}

/**************** BASIC FUNCTIONS *********/
void ReadEncoders(){
  left_ticks_i32  = left_encoder.read();
  right_ticks_i32 = right_encoder.read();
}

void RotateMotors(){
  uint8_t right_cmd = (uint8_t)(constrain(abs(right_motor_cmd), MIN_MOTOR_CMD, MAX_MOTOR_CMD));
  uint8_t left_cmd  = (uint8_t)(constrain(abs(left_motor_cmd),  MIN_MOTOR_CMD, MAX_MOTOR_CMD));

  if (right_motor_cmd >= 0){
    analogWrite(RIGHTMOTOR_FWD_PWM, right_cmd);
    analogWrite(RIGHTMOTOR_BWD_PWM, 0);
  } else {
    analogWrite(RIGHTMOTOR_FWD_PWM, 0);
    analogWrite(RIGHTMOTOR_BWD_PWM, right_cmd);
  }

  if (left_motor_cmd >= 0){
    analogWrite(LEFTMOTOR_FWD_PWM, left_cmd);
    analogWrite(LEFTMOTOR_BWD_PWM, 0);
  } else {
    analogWrite(LEFTMOTOR_FWD_PWM, 0);
    analogWrite(LEFTMOTOR_BWD_PWM, left_cmd);
  }
}

void SetServoAngle() {
  filtered_camera_angle = (last_camera_angle * CAMERA_SMOOTHING) + (filtered_camera_angle * (1.0 - CAMERA_SMOOTHING));

  float L = WHEEL_BASE_MM / 1000.0;
  float error_deg = filtered_camera_angle - 87.0;
  float abs_error = abs(error_deg);

  float active_K = (abs_error > GAIN_THRESHOLD) ? K_SHARP : K_STRAIGHT;

  float direction_multiplier = 1.0;
  float heading_error_rad = radians(error_deg * direction_multiplier);

  float curvature    = active_K * heading_error_rad;
  float steering_rad = atan(L * curvature);
  float steering_deg = degrees(steering_rad);

  float servo_angle = 87.0 + steering_deg;

  if (servo_angle >= 87) servo_angle = 87 + (servo_angle - 87) * STEERING_KP;
  else                   servo_angle = 87 - (87 - servo_angle) * STEERING_KP;

  servo_angle = 180 - servo_angle;
  servo_angle = constrain(servo_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  servo_angle_cmd_deg = (int16_t)servo_angle;
  steer_servo.write(servo_angle_cmd_deg);
}

void StopMotors(){
  analogWrite(RIGHTMOTOR_FWD_PWM, 0);
  analogWrite(RIGHTMOTOR_BWD_PWM, 0);
  analogWrite(LEFTMOTOR_FWD_PWM,  0);
  analogWrite(LEFTMOTOR_BWD_PWM,  0);
}

/**************** ODOMETRY FUNCTIONS *********/
void ConvertTicksToDistance(){
  left_wheel_distance_mm  = (left_ticks_i32  * M_PI * LEFT_WHEEL_DIAMETER_MM)  / LEFT_ENCODER_CPR;
  right_wheel_distance_mm = (right_ticks_i32 * M_PI * RIGHT_WHEEL_DIAMETER_MM) / RIGHT_ENCODER_CPR;
  robot_distance_mm = (left_wheel_distance_mm + right_wheel_distance_mm) / 2.0;
}

void ConvertDistanceToVel(){
  double raw_left_vel  = 1000 * (left_wheel_distance_mm  - left_wheel_dist_prev_vel_calc)  / VELOCITY_CALC_DT_MS;
  double raw_right_vel = 1000 * (right_wheel_distance_mm - right_wheel_dist_prev_vel_calc) / VELOCITY_CALC_DT_MS;

  left_vel_filtered  = left_vel_filtered  * 0.8 + raw_left_vel  * 0.2;
  right_vel_filtered = right_vel_filtered * 0.8 + raw_right_vel * 0.2;

  left_wheel_curr_vel_mm_s  = left_vel_filtered;
  right_wheel_curr_vel_mm_s = right_vel_filtered;

  left_wheel_dist_prev_vel_calc  = left_wheel_distance_mm;
  right_wheel_dist_prev_vel_calc = right_wheel_distance_mm;

  robot_curr_vel_mm_s = (right_wheel_curr_vel_mm_s + left_wheel_curr_vel_mm_s) / 2.0;
}

/**************** CONTROLLER FUNCTIONS *********/
void CalculateDistanceError(){
  distance_error_mm = distance_setpoint_mm - robot_distance_mm;
}

void CalculateVelError(){
  left_motor_vel_last_error_mm_s  = left_motor_vel_error_mm_s;
  right_motor_vel_last_error_mm_s = right_motor_vel_error_mm_s;

  left_motor_vel_error_mm_s  = left_motor_vel_setpoint_mm_s  - left_wheel_curr_vel_mm_s;
  right_motor_vel_error_mm_s = right_motor_vel_setpoint_mm_s - right_wheel_curr_vel_mm_s;
  robot_vel_error_mm_s       = robot_vel_setpoint_mm_s       - robot_curr_vel_mm_s;
}

void CalculateVelPID(){
  left_motor_vel_error_sum_mm_s  += left_motor_vel_error_mm_s;
  right_motor_vel_error_sum_mm_s += right_motor_vel_error_mm_s;

  left_motor_vel_error_sum_mm_s  = constrain(left_motor_vel_error_sum_mm_s,  -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);
  right_motor_vel_error_sum_mm_s = constrain(right_motor_vel_error_sum_mm_s, -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);

  float left_motor_vel_error_sub_mm_s  = left_motor_vel_error_mm_s  - left_motor_vel_last_error_mm_s;
  float right_motor_vel_error_sub_mm_s = right_motor_vel_error_mm_s - right_motor_vel_last_error_mm_s;

  left_motor_vel_pid_output = (left_motor_vel_error_mm_s  * left_vel_kp) +
                              (left_motor_vel_error_sum_mm_s  * left_vel_ki) +
                              (left_motor_vel_error_sub_mm_s  * left_vel_kd);

  right_motor_vel_pid_output = (right_motor_vel_error_mm_s * right_vel_kp) +
                               (right_motor_vel_error_sum_mm_s * right_vel_ki) +
                               (right_motor_vel_error_sub_mm_s * right_vel_kd);

  right_motor_cmd = constrain(right_motor_vel_pid_output, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  left_motor_cmd  = constrain(left_motor_vel_pid_output,  -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
}

void CalculateOrientationError(){
  orientation_last_error_deg = orientation_error_deg;
  orientation_error_deg = orientation_setpoint_deg - curr_orientation_deg;
}

void CalculateSteeringPID(){
  orientation_error_sum_deg = constrain(orientation_error_sum_deg + orientation_error_deg,
                                        -MAX_STEERING_ERROR_SUM, MAX_STEERING_ERROR_SUM);
  float orientation_error_sub_deg = orientation_error_deg - orientation_last_error_deg;

  servo_angle_pid_output = (orientation_error_deg     * steering_kp) +
                           (orientation_error_sum_deg  * steering_ki) +
                           (orientation_error_sub_deg  * steering_kd);

  servo_angle_cmd_deg = constrain(servo_angle_pid_output, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
}