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

#define DEBUG 1


#define UART_TX 1
#define UART_RX 0

#define LEFT_ENC_CH1 3
#define LEFT_ENC_CH2 2

#define RIGHT_ENC_CH1 5
#define RIGHT_ENC_CH2 4

#define RIGHTMOTOR_FWD_PWM 23 //Forward PWM
#define RIGHTMOTOR_BWD_PWM 22 //Backward PWM

#define LEFTMOTOR_FWD_PWM 6 //Forward PWM
#define LEFTMOTOR_BWD_PWM 7 //Backward PWM

#define TIM1_PIN 8
#define TIM3_PIN 9

#define SERVO_PIN 17  

#define SPI_CS_PIN 10   //ya turki badel pinet spi ll camera teensy 4.0
#define SPI_MOSI_PIN 11  
#define SPI_MISO_PIN 12
#define SPI_SCLK_PIN 13

#define XSHUT_1 14
#define XSHUT_2 15
#define XSHUT_3 16

#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

/* ToF addresses */
#define TOF_ADDR_1 0x30
#define TOF_ADDR_2 0x31
#define TOF_ADDR_3 0x32

/***************** CONTROLLER DEFINES **************** */
//PID DEFINES
#define RIGHT_VEL_KP 0.25 //0.1
#define RIGHT_VEL_KI 0.007 //0.001
#define RIGHT_VEL_KD 0.0 //0.0

#define LEFT_VEL_KP 0.025
#define LEFT_VEL_KI 0.007
#define LEFT_VEL_KD 0.0

#define STEERING_KP 1.0
#define STEERING_KI 0.0
#define STEERING_KD 0.0

#define MAX_SERVO_ANGLE 127 //125 //imin
#define MIN_SERVO_ANGLE 47 //47 //55 //isar
#define MAX_MOTOR_CMD 255
#define MIN_MOTOR_CMD 0
#define MAX_STEERING_ERROR_SUM 120  // Prevent integral windup
#define MAX_VEL_ERROR_SUM 35000

#define WHEEL_GAIN  1.000
#define CONTROL_LOOP_DT_MS 5  // 5ms = 0.005 seconds (200Hz control loop from Timer1)
#define VELOCITY_CALC_DT_MS 5 
#define STOP_DISTANCE 650

/****************  ODOMETRY DEFINES ************* */
#define LEFT_ENCODER_CPR 408
#define RIGHT_ENCODER_CPR 408
#define LEFT_WHEEL_DIAMETER_MM 67.58 //arbitrary number //65 //91.77 //72.19
#define RIGHT_WHEEL_DIAMETER_MM 67.58 //65 //99.32
#define WHEEL_BASE_MM 194 //distance between wheels
#define SERVO_INIT_ANGLE 87 //87

float K_STRAIGHT = 5.0;//2.5  // Gain for small corrections
float K_SHARP = 8;     // Gain for sharp turns
float GAIN_THRESHOLD = 33.0; // Angle (deg) where we start switching to high gain
float CAMERA_SMOOTHING = 0.3; // 0 to 1. Lower is smoother, higher is more responsive.

float filtered_camera_angle = 87.0;
/********************** ODOMETRY VARIABLES ********************* */
volatile double left_wheel_curr_vel_mm_s, right_wheel_curr_vel_mm_s, robot_curr_vel_mm_s;
volatile double prev_left_wheel_dist_mm, prev_right_wheel_dist_mm, prev_robot_dist_mm;
volatile double left_wheel_distance_mm, right_wheel_distance_mm, robot_distance_mm;
volatile double curr_orientation_deg;
volatile double left_vel_filtered, right_vel_filtered;

volatile double left_wheel_dist_prev_vel_calc = 0;
volatile double right_wheel_dist_prev_vel_calc = 0;
volatile uint32_t last_vel_calc_ms = 0;
/************************ PID VARIABLES *************  */
volatile float right_vel_kp = RIGHT_VEL_KP, right_vel_ki = RIGHT_VEL_KI, right_vel_kd = RIGHT_VEL_KD;
volatile float left_vel_kp = LEFT_VEL_KP, left_vel_ki = LEFT_VEL_KI, left_vel_kd = LEFT_VEL_KD;
volatile float steering_kp = STEERING_KP, steering_ki = STEERING_KI, steering_kd = STEERING_KD;
volatile uint8_t K_heading = 2;
/**************** ACTUATORS VARIABLES ********** */
volatile int32_t right_motor_cmd, left_motor_cmd;
volatile int16_t servo_angle_cmd_deg = 93 ; //in deg

/**************** SENSORS VARIABLES ************* */
volatile int32_t left_ticks_i32, right_ticks_i32;

/**************** CONTROL VARIABLES ******* */
//VELOCITY CONTROL
volatile double left_motor_vel_setpoint_mm_s, right_motor_vel_setpoint_mm_s, robot_vel_setpoint_mm_s; //desired velocity to reach
volatile double prev_left_motor_vel_setpoint_mm_s, prev_right_motor_vel_setpoint_mm_s; // Track previous setpoints
volatile double left_motor_vel_error_mm_s, right_motor_vel_error_mm_s, robot_vel_error_mm_s;

volatile double left_motor_vel_pid_output, right_motor_vel_pid_output;
volatile double left_motor_vel_error_sum_mm_s, right_motor_vel_error_sum_mm_s;  
volatile double left_motor_vel_last_error_mm_s, right_motor_vel_last_error_mm_s;  
  
//STEERING CONTROL
volatile float orientation_setpoint_deg, orientation_error_deg, orientation_error_sum_deg;
volatile float orientation_last_error_deg;
volatile float servo_angle_pid_output;
//DISTANCE  CONTROL
volatile float distance_setpoint_mm, prev_robot_distance_mm;
volatile float distance_error_mm;
volatile bool distance_control_enable = false;
volatile bool distance_reached = false;
//EMERGENCY STOP
volatile bool emergency_stop_enable = false;

// camera angle 
volatile float last_camera_angle;
volatile float last_camera_angle_updated;
volatile uint32_t servo_wait = 0;
VL53L0X_RangingMeasurementData_t tof_measure;
float left_tof_distance, center_tof_distance, right_tof_distance;
uint32_t last_tof_test = 0; // track last read
const uint32_t TOF_INTERVAL_MS = 100; // ~10 Hz reading
bool cube_detected = false;
ToFFilter tofFilter;
/********* INSTANCES ******** */
QuadEncoder left_encoder(1, LEFT_ENC_CH1, LEFT_ENC_CH2);
QuadEncoder right_encoder(2, RIGHT_ENC_CH1, RIGHT_ENC_CH2);
Servo  steer_servo;
Vision vision;
/*********** DEBUG VARIABLES ****** */
uint32_t last_debug = 0;
/*TOF */
/* ToF Init */
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof2 = Adafruit_VL53L0X();
Adafruit_VL53L0X tof3 = Adafruit_VL53L0X();
/************** FUNCTIONS DECLARATIONS  ******* */
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
void GetOrientation(); // using encoders for now until camera code comes
void VelOdomRoutine();

void parseTuningValues(String data);
void checkUARTForPID();
void SendStatusToESP32();

void CalculateDistanceError();//used for tuning only
void StopMotors();

void EmptyFunction(){
  
}

void NavRoutine(){
  VelOdomRoutine();
  //GetOrientation();
  VelControllerRoutine();    
  if (cube_detected == true){
    StopMotors();
  }
  else {
    RotateMotors();
  }
  //CalculateOrientationError();
  //CalculateSteeringPID();
  if (millis() - servo_wait >=750){
    SetServoAngle();
  }
  //checkUARTForPID();  
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

void softwareReset()
{
  SCB_AIRCR = 0x05FA0004;
}

void readToFsNonBlocking() {
    if (millis() - last_tof_test >= TOF_INTERVAL_MS) {
        uint16_t distance;

        // ----- ToF1 -----
        if (tof1.isRangeComplete()) {
            distance = tof1.readRange();
            if (!tof1.timeoutOccurred()) {
                center_tof_distance = tofFilter.filter(distance)*1000;//convert to mm
                //center_tof_distance = distance;

            } // else handle timeout if needed
        }

        // ----- ToF2 -----
        if (tof2.isRangeComplete()) {
            distance = tof2.readRange();
            if (!tof2.timeoutOccurred()) {
                left_tof_distance = distance;
            }
        }

        // ----- ToF3 -----
        if (tof3.isRangeComplete()) {
            distance = tof3.readRange();
            if (!tof3.timeoutOccurred()) {
                right_tof_distance = distance;
            }
        }
        // if ((left_tof_distance < STOP_DISTANCE) ||
        // (right_tof_distance < STOP_DISTANCE) ||
        // (center_tof_distance < STOP_DISTANCE)){
        //   cube_detected = true;
        // }
        // else if ((left_tof_distance > STOP_DISTANCE) &&
        // (right_tof_distance > STOP_DISTANCE) &&
        // (center_tof_distance > STOP_DISTANCE)){
        //   cube_detected = false;
        // }
        if (center_tof_distance < STOP_DISTANCE){
          cube_detected = true;
        }
        else if (center_tof_distance > STOP_DISTANCE){
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



void setup() {
  /****************  ENCODERS INIT ************* */
  vision.begin();
  // Initialize left encoder
  delay(3000);
  vision.pixy.setLamp(1, 1);
  left_encoder.setInitConfig();
  left_encoder.init();

  // Initialize right encoder
  right_encoder.setInitConfig();
  right_encoder.init();

  left_encoder.write(0);
  right_encoder.write(0);
  servo_wait = millis();
  /**************** TIMERS INIT ************* */
  // Timer1.initialize(CONTROL_LOOP_DT_MS*1000);          // set period in µs //5000
  // //Timer1.attachInterrupt(NavRoutine);  // attach the interrupt function

  /****************  SERVO INIT ************* */
  steer_servo.attach(SERVO_PIN);
  steer_servo.write(SERVO_INIT_ANGLE);
  Serial.begin(115200);
  Serial1.begin(115200);
    /**************** TIMERS INIT *********** */
  Timer1.initialize(CONTROL_LOOP_DT_MS*1000);          // set period in µs //5000
  Timer1.attachInterrupt(NavRoutine);  // attach the interrupt function
   /*ToF Init */
  Wire.begin();
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.setClock(400000); // Fast I2C
  
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  // ---------------- SENSOR 1 ----------------
  digitalWrite(XSHUT_1, HIGH);
  delay(10);

  if (!tof1.begin(0x29, &Wire)) {
    Serial.println("Failed to boot ToF 1");
    while (1);
  }
  tof1.setAddress(TOF_ADDR_1);
  Serial.println("ToF 1 initialized");

  // ---------------- SENSOR 2 ----------------
  digitalWrite(XSHUT_2, HIGH);
  delay(10);

  if (!tof2.begin(0x29, &Wire)) {
    Serial.println("Failed to boot ToF 2");
    while (1);
  }
  tof2.setAddress(TOF_ADDR_2);
  Serial.println("ToF 2 initialized");

  // ---------------- SENSOR 3 ----------------
  digitalWrite(XSHUT_3, HIGH);
  delay(10);

  if (!tof3.begin(0x29, &Wire)) {
    Serial.println("Failed to boot ToF 3");
    while (1);
  }
  tof3.setAddress(TOF_ADDR_3);
  tof1.startRangeContinuous(50);
  tof2.startRangeContinuous(50);
  tof3.startRangeContinuous(50);
  Serial.println("ToF 3 initialized");
  Serial.println("All ToF sensors ready");
  /*Filter Initialisation*/
  tofFilter.setOffset(15);
  tofFilter.setRangeLimits(20, 20000);
  tofFilter.setPublishInterval(1000/TOF_INTERVAL_MS); // 2 Hz max

  left_motor_vel_setpoint_mm_s = 1000; //750
  right_motor_vel_setpoint_mm_s = 1000; //750
  
}

void loop() {
  // Check for commands from ESP32 via Serial1
  
  
  // Send status to ESP32 every 100ms
  static uint32_t last_status_send = 0;
  if (millis() - last_status_send > 100) {
    //SendStatusToESP32();
    last_status_send = millis();
  }
  
  // // Manual setpoint input from Serial Monitor for testing

  // if (millis() - last_debug > 10) {
  //   // Teleplot format: >variable_name:value
  //   Serial.print(">enc right:");
  //   Serial.println(right_ticks_i32);
  //   Serial.print(">enc left:");
  //   Serial.println(left_ticks_i32);

  //   Serial.print(">left distance:");
  //   Serial.println(left_wheel_distance_mm);
  //   Serial.print(">right distance:");
  //   Serial.println(right_wheel_distance_mm);

  //   Serial.print(">right_velocity:");
  //   Serial.println(right_wheel_curr_vel_mm_s);
    
  //   Serial.print(">right_cmd:");
  //   Serial.println(right_motor_cmd);

  //   Serial.print(">velocity_setpoint:");
  //   Serial.println(right_motor_vel_setpoint_mm_s);

  //   Serial.print(">left_velocity:");
  //   Serial.println(left_wheel_curr_vel_mm_s);
    
  //   Serial.print(">left_cmd:");
  //   Serial.println(left_motor_cmd);
    
  //   Serial.print(">left_cmd:");
  //   Serial.println(left_motor_vel_error_sum_mm_s);

  //   //Serial.print("distance error");Serial.println(distance_error_mm);
  //   last_debug = millis();
  //   //delay(100);
  // }
  readToFsNonBlocking();
  String mode;
  float distance;
  last_camera_angle = vision.calculate_steering_angle(mode, distance);
  last_camera_angle=180-last_camera_angle; 
  //Serial.print(">camera_angle:");
  //Serial.println(last_camera_angle_updated);
 
}

/****************  BASIC FUNCTIONS *********** */
void ReadEncoders(){
  left_ticks_i32 = left_encoder.read();
  right_ticks_i32 = right_encoder.read();
} 
/*
void GetOrientation(){
  String mode;
  float distance;
  last_camera_angle = vision.calculate_steering_angle(mode, distance);
}
  */


void RotateMotors(){
  uint8_t right_cmd =(uint8_t)(constrain(abs(right_motor_cmd), MIN_MOTOR_CMD, MAX_MOTOR_CMD));
  uint8_t left_cmd = (uint8_t)(constrain(abs(left_motor_cmd), MIN_MOTOR_CMD, MAX_MOTOR_CMD));
  if (right_motor_cmd>=0){
    analogWrite(RIGHTMOTOR_FWD_PWM, right_cmd);
    analogWrite(RIGHTMOTOR_BWD_PWM, 0);
  } 
  else {
    analogWrite(RIGHTMOTOR_FWD_PWM, 0);
    analogWrite(RIGHTMOTOR_BWD_PWM, right_cmd);
  }
  if (left_motor_cmd>=0){
    analogWrite(LEFTMOTOR_FWD_PWM, left_cmd);
    analogWrite(LEFTMOTOR_BWD_PWM, 0);
  }
  else {
    analogWrite(LEFTMOTOR_FWD_PWM, 0);
    analogWrite(LEFTMOTOR_BWD_PWM, left_cmd);
  }
}

/*
before viision code:
void SetServoAngle(){
  steer_servo.write(servo_angle_cmd_deg);
}
*/
void SetServoAngle() {
  // 1. Filter the camera input to stop the "jitters"
  filtered_camera_angle = (last_camera_angle * CAMERA_SMOOTHING) + (filtered_camera_angle * (1.0 - CAMERA_SMOOTHING));

  float L = WHEEL_BASE_MM / 1000.0; 
  float error_deg = filtered_camera_angle - 87.0;
  float abs_error = abs(error_deg);

  // 2. Dynamic K-Gain Selection
  // If the error is large, use K_SHARP; otherwise use K_STRAIGHT
  float active_K = (abs_error > GAIN_THRESHOLD) ? K_SHARP : K_STRAIGHT;

  // 3. Direction Multiplier 
  // IMPORTANT: If it snaps to the WRONG side, change this to -1.0
  float direction_multiplier = 1.0; 
  float heading_error_rad = radians(error_deg * direction_multiplier);

  // 4. Geometry Math
  float curvature = active_K * heading_error_rad;
  float steering_rad = atan(L * curvature);
  float steering_deg = degrees(steering_rad);
  
  // 5. Apply to Servo
  float servo_angle = 87.0 + steering_deg;

  // Use your STEERING_KP (from defines) to scale the final output
  if (servo_angle >= 87) servo_angle = 87 + (servo_angle - 87) * STEERING_KP;
  else servo_angle = 87 - (87 - servo_angle) * STEERING_KP;
  servo_angle = 180 -servo_angle;
  servo_angle = constrain(servo_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  servo_angle_cmd_deg = (int16_t)servo_angle; 
  //steer_servo.write(servo_angle_cmd_deg);
  //last_camera_angle*
  //float last_camera_angle_updated_local = 180 - last_camera_angle_updated;
  //last_camera_angle_updated_local = constrain(last_camera_angle_updated, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
  //servo_angle_cmd_deg = (int16_t)last_camera_angle_updated_local; 
  steer_servo.write(servo_angle_cmd_deg);
}


void StopMotors(){
  analogWrite(RIGHTMOTOR_FWD_PWM, 0);
  analogWrite(RIGHTMOTOR_BWD_PWM, 0);
  analogWrite(LEFTMOTOR_FWD_PWM, 0);
  analogWrite(LEFTMOTOR_BWD_PWM, 0);
}

/****************  ODOMETRY FUNCTIONS *********** */

void ConvertTicksToDistance(){
  left_wheel_distance_mm = (left_ticks_i32 * M_PI * LEFT_WHEEL_DIAMETER_MM) / LEFT_ENCODER_CPR;
  right_wheel_distance_mm = (right_ticks_i32 * M_PI * RIGHT_WHEEL_DIAMETER_MM) / RIGHT_ENCODER_CPR ;
  robot_distance_mm = (left_wheel_distance_mm + right_wheel_distance_mm)/2.0;
}

void ConvertDistanceToVel(){
  double raw_left_vel = 1000*(left_wheel_distance_mm - left_wheel_dist_prev_vel_calc)/(VELOCITY_CALC_DT_MS);
  double raw_right_vel = 1000*(right_wheel_distance_mm - right_wheel_dist_prev_vel_calc)/(VELOCITY_CALC_DT_MS);
  
  left_vel_filtered = (left_vel_filtered * 0.8 + (raw_left_vel * 0.2));
  right_vel_filtered = (right_vel_filtered * 0.8 + (raw_right_vel * 0.2));

  left_wheel_curr_vel_mm_s = left_vel_filtered;
  right_wheel_curr_vel_mm_s = right_vel_filtered;
  
  left_wheel_dist_prev_vel_calc = left_wheel_distance_mm;
  right_wheel_dist_prev_vel_calc = right_wheel_distance_mm;
  
  robot_curr_vel_mm_s = (right_wheel_curr_vel_mm_s + left_wheel_curr_vel_mm_s) / 2.0;
}

/****************  CONTROLLER FUNCTIONS *********** */

void CalculateDistanceError(){//used for tuning only
  distance_error_mm = distance_setpoint_mm - robot_distance_mm; 
}

void CalculateVelError(){
  //store previous values
  left_motor_vel_last_error_mm_s = left_motor_vel_error_mm_s;
  right_motor_vel_last_error_mm_s  = right_motor_vel_error_mm_s;
  //update error
  left_motor_vel_error_mm_s = left_motor_vel_setpoint_mm_s - left_wheel_curr_vel_mm_s;
  right_motor_vel_error_mm_s = right_motor_vel_setpoint_mm_s - right_wheel_curr_vel_mm_s;
  robot_vel_error_mm_s = robot_vel_setpoint_mm_s - robot_curr_vel_mm_s;
}

void CalculateVelPID(){
  
  // Accumulate integral
  left_motor_vel_error_sum_mm_s += left_motor_vel_error_mm_s;
  right_motor_vel_error_sum_mm_s += right_motor_vel_error_mm_s;
  
  // Enable integral windup protection to prevent unbounded growth
  left_motor_vel_error_sum_mm_s = constrain(left_motor_vel_error_sum_mm_s, -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);
  right_motor_vel_error_sum_mm_s = constrain(right_motor_vel_error_sum_mm_s, -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);
  
  // FIXED: Divide derivative by dt
  float left_motor_vel_error_sub_mm_s = (left_motor_vel_error_mm_s - left_motor_vel_last_error_mm_s) ;
  float right_motor_vel_error_sub_mm_s = (right_motor_vel_error_mm_s - right_motor_vel_last_error_mm_s) ;
  
  // Calculate PID output
  left_motor_vel_pid_output = (left_motor_vel_error_mm_s * left_vel_kp) +
                              (left_motor_vel_error_sum_mm_s * left_vel_ki) +
                              (left_motor_vel_error_sub_mm_s * left_vel_kd);

  right_motor_vel_pid_output = (right_motor_vel_error_mm_s * right_vel_kp) +
                               (right_motor_vel_error_sum_mm_s * right_vel_ki) +
                               (right_motor_vel_error_sub_mm_s * right_vel_kd);    
                              
  // FIXED: Add output saturation to prevent PWM overflow
  right_motor_cmd = constrain(right_motor_vel_pid_output, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);
  left_motor_cmd = constrain(left_motor_vel_pid_output, -MAX_MOTOR_CMD, MAX_MOTOR_CMD);                          
} 

void CalculateOrientationError(){
  //store prev error
  orientation_last_error_deg = orientation_error_deg;
  //update error
  orientation_error_deg = orientation_setpoint_deg - curr_orientation_deg;
}

void CalculateSteeringPID(){
  orientation_error_sum_deg = constrain(orientation_error_sum_deg + orientation_error_deg, 
                                       -MAX_STEERING_ERROR_SUM, MAX_STEERING_ERROR_SUM);
  float orientation_error_sub_deg = orientation_error_deg - orientation_last_error_deg;
  servo_angle_pid_output = (orientation_error_deg*steering_kp)+
                           (orientation_error_sum_deg*steering_ki)+
                           (orientation_error_sub_deg*steering_kd);
  
  //servo cmd = pid_output (+ constraint)
  servo_angle_cmd_deg = constrain(servo_angle_pid_output, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
}

void parseTuningValues(String data) {
  // Parse new format: "right_kp,right_ki,right_kd,left_kp,left_ki,left_kd,steer_kp,steer_ki,steer_kd,right_velocity,left_velocity,distance,emergency_stop,distance_mode"
  int commas[13];
  int index = 0;
  
  // Find all 13 comma positions (14 values)
  for (unsigned int i = 0; i < data.length() && index < 13; i++) {
    if (data[i] == ',') {
      commas[index] = i;
      index++;
    }
  }
  
  if (index >= 13) {
    // Parse RIGHT motor PID
    // prev_robot_distance_mm = robot_distance_mm;
    // right_vel_kp = data.substring(0, commas[0]).toFloat();
    // right_vel_ki = data.substring(commas[0]+1, commas[1]).toFloat();
    // right_vel_kd = data.substring(commas[1]+1, commas[2]).toFloat();
    
    // // Parse LEFT motor PID
    // left_vel_kp = data.substring(commas[2]+1, commas[3]).toFloat();
    // left_vel_ki = data.substring(commas[3]+1, commas[4]).toFloat();
    // left_vel_kd = data.substring(commas[4]+1, commas[5]).toFloat();
    
    // // Parse Steering PID
    // steering_kp = data.substring(commas[5]+1, commas[6]).toFloat();
    // steering_ki = data.substring(commas[6]+1, commas[7]).toFloat();
    // steering_kd = data.substring(commas[7]+1, commas[8]).toFloat();
    
    // // Parse velocity setpoints and distance
    // right_motor_vel_setpoint_mm_s = data.substring(commas[8]+1, commas[9]).toFloat();
    // left_motor_vel_setpoint_mm_s = data.substring(commas[9]+1, commas[10]).toFloat();
    // distance_setpoint_mm = (data.substring(commas[10]+1, commas[11]).toFloat()) + prev_robot_distance_mm;
    
    // Parse control flags
    int emergency = data.substring(commas[11]+1, commas[12]).toInt();
    emergency_stop_enable = (emergency == 1);
    
    // int dist_mode = data.substring(commas[12]+1).toInt();
    // distance_control_enable = (dist_mode == 1);
    
    //right_motor_vel_error_sum_mm_s = 0.0;
    //left_motor_vel_error_sum_mm_s = 0.0;

    Serial.println("PID Updated:");
    Serial.print("Right: Kp="); Serial.print(right_vel_kp,5);
    Serial.print(" Ki="); Serial.print(right_vel_ki,5);
    Serial.print(" Kd="); Serial.println(right_vel_kd,5);
    Serial.print("Left: Kp="); Serial.print(left_vel_kp,5);
    Serial.print(" Ki="); Serial.print(left_vel_ki,5);
    Serial.print(" Kd="); Serial.println(left_vel_kd,5);
    Serial.print("Distance mode: ");
    Serial.println(distance_control_enable ? "ENABLED" : "DISABLED");
    Serial.print("Distance: "); Serial.println(data.substring(commas[10]+1, commas[11]).toFloat(),3);
    distance_reached = false;
  }
}

void checkUARTForPID() {
  if (Serial1.available()) {
    String pidString = Serial1.readStringUntil('\n');
    parseTuningValues(pidString);
  }
}

void SendStatusToESP32() {
  // Send current robot status: "robot_distance,left_velocity,right_velocity,left_distance,right_distance"
  String statusData = String(right_motor_vel_error_sum_mm_s) + "," +
                     String(right_motor_cmd) + "," +
                     String(left_wheel_curr_vel_mm_s, 2) + "," +
                     String(right_wheel_curr_vel_mm_s, 2) + "," +
                     String(right_motor_vel_setpoint_mm_s, 2) + "\n";
                    
  Serial1.print(statusData);
   
}