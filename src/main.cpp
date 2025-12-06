//THIS BRANCH CODE IS FOR RUNNING THE ROBOT IN OFFICIAL MATCH (NOT FOR TUNING)
//DIDN'T UNDEGRATE THE (ASSERVISSEMENT PAR ROUE) WILL DO THAT IN THE NEXT COMMIT
//STILL NO CAMERA CODE (SETPOINTS ASSIGNMENT ALGORITHM IN GENERAL)
#include <Arduino.h>
#include <PWMServo.h>
#include <TimerOne.h>
#include <QuadEncoder.h>

#define LEFT_ENC_CH1 2
#define LEFT_ENC_CH2 3

#define RIGHT_ENC_CH1 4
#define RIGHT_ENC_CH2 5

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

#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

/***************** CONTROLLER DEFINES ****************** */
//PID DEFINES
#define VEL_KP 1.0
#define VEL_KI 0.00001
#define VEL_KD 0.5

#define STEERING_KP 1.0
#define STEERING_KI 0.00001
#define STEERING_KD 0.5

#define MAX_SERVO_ANGLE 150
#define MIN_SERVO_ANGLE 30
#define MAX_MOTOR_CMD 255
#define MIN_MOTOR_CMD 0
#define MAX_STEERING_ERROR_SUM 120  // Prevent integral windup
#define MAX_VEL_ERROR_SUM 1000

#define WHEEL_GAIN  0.995
/****************  ODOMETRY DEFINES *************** */
#define LEFT_ENCODER_CPR 280
#define RIGHT_ENCODER_CPR 280
#define LEFT_WHEEL_DIAMETER_MM 50 //arbitrary number
#define RIGHT_WHEEL_DIAMETER_MM 50

/********************** ODOMETRY VARIABLES *********************** */
volatile float left_wheel_curr_vel_mm_s, right_wheel_curr_vel_mm_s, robot_curr_vel_mm_s;
volatile float prev_left_wheel_dist_mm, prev_right_wheel_dist_mm, prev_robot_dist_mm;
volatile float left_wheel_distance_mm, right_wheel_distance_mm, robot_distance_mm;
volatile float curr_orientation_deg;
/**************** ACTUATORS VARIABLES ************ */
volatile int32_t right_motor_cmd, left_motor_cmd;
volatile int16_t servo_angle_cmd_deg; //in deg

/**************** SENSORS VARIABLES *************** */
volatile int32_t left_ticks_i32, right_ticks_i32;

/**************** CONTROL VARIABLES ********* */
//VELOCITY CONTROL
volatile float left_motor_vel_setpoint_mm_s, right_motor_vel_setpoint_mm_s, robot_vel_setpoint_mm_s; //desired velocity to reach
volatile float left_motor_vel_error_mm_s, right_motor_vel_error_mm_s, robot_vel_error_mm_s;

volatile float left_motor_vel_pid_output, right_motor_vel_pid_output;
volatile float left_motor_vel_error_sum_mm_s, right_motor_vel_error_sum_mm_s;  
volatile float left_motor_vel_last_error_mm_s, right_motor_vel_last_error_mm_s;  
  
//STEERING CONTROL
volatile float orientation_setpoint_deg, orientation_error_deg, orientation_error_sum_deg;
volatile float orientation_last_error_deg;
volatile float servo_angle_pid_output;

/********* INSTANCES ********** */
QuadEncoder left_encoder(1, LEFT_ENC_CH1, LEFT_ENC_CH2);
QuadEncoder right_encoder(2, RIGHT_ENC_CH1, RIGHT_ENC_CH2);

PWMServo  steer_servo;

/*********** DEBUG VARIABLES ******** */
uint32_t last_debug = 0;

/************** FUNCTIONS DECLARATIONS  ********* */
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
void GetOrientation();
void VelOdomRoutine();


void NavRoutine(){
  //velocity routine
  VelOdomRoutine();
  VelControllerRoutine();
  RotateMotors();

  //steering routine
  GetOrientation();
  CalculateOrientationError();
  CalculateSteeringPID();
  SetServoAngle();
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

void setup() {
  /****************  ENCODERS INIT *************** */
  // Initialize left encoder
  left_encoder.setInitConfig();
  left_encoder.init();

  // Initialize right encoder
  right_encoder.setInitConfig();
  right_encoder.init();

  left_encoder.write(0);
  right_encoder.write(0);

  /**************** TIMERS INIT *************** */
  Timer1.initialize(5000);          // set period in µs
  Timer1.attachInterrupt(NavRoutine);  // attach the interrupt function

  /****************  SERVO INIT *************** */
  steer_servo.attach(SERVO_PIN);

  Serial.begin(115200);
}

void loop() {
  if (millis() - last_debug > 100) { // Every 100ms
    Serial.print("Vel: "); Serial.print(robot_curr_vel_mm_s);
    Serial.print(" Orient: "); Serial.print(curr_orientation_deg);
    Serial.print(" Servo: "); Serial.println(servo_angle_cmd_deg);
    last_debug = millis();
  }
}

/****************  BASIC FUNCTIONS *************** */
void ReadEncoders(){
  left_ticks_i32 = left_encoder.read();
  right_ticks_i32 = right_encoder.read();
} 

void GetOrientation(){
  //get orientation from camera 
  curr_orientation_deg = 50.0; //random number
}

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

void SetServoAngle(){
  steer_servo.write(servo_angle_cmd_deg);
}

/****************  ODOMETRY FUNCTIONS *************** */

void ConvertTicksToDistance(){
  //store previous values
  prev_left_wheel_dist_mm = left_wheel_distance_mm;
  prev_right_wheel_dist_mm = right_wheel_distance_mm;
  //update values
  left_wheel_distance_mm = (left_ticks_i32 * M_PI * LEFT_WHEEL_DIAMETER_MM) / LEFT_ENCODER_CPR;
  right_wheel_distance_mm = (right_ticks_i32 * M_PI * RIGHT_WHEEL_DIAMETER_MM) / RIGHT_ENCODER_CPR ;
  robot_distance_mm = (left_wheel_distance_mm + right_wheel_distance_mm)/2.0;
}

void ConvertDistanceToVel(){
  left_wheel_curr_vel_mm_s = left_wheel_distance_mm - prev_left_wheel_dist_mm;
  right_wheel_curr_vel_mm_s = right_wheel_distance_mm - prev_right_wheel_dist_mm;
  robot_curr_vel_mm_s = (right_wheel_curr_vel_mm_s + left_wheel_curr_vel_mm_s)/2.0;
}

/****************  CONTROLLER FUNCTIONS *************** */

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
  //calculate sum and derivative
  left_motor_vel_error_sum_mm_s = constrain(left_motor_vel_error_sum_mm_s + left_motor_vel_error_mm_s,-MAX_VEL_ERROR_SUM,MAX_VEL_ERROR_SUM);
  right_motor_vel_error_sum_mm_s = constrain(right_motor_vel_error_sum_mm_s + right_motor_vel_error_mm_s,-MAX_VEL_ERROR_SUM,MAX_VEL_ERROR_SUM);
  float left_motor_vel_error_sub_mm_s = left_motor_vel_error_mm_s - left_motor_vel_last_error_mm_s;
  float right_motor_vel_error_sub_mm_s = right_motor_vel_error_mm_s - right_motor_vel_last_error_mm_s;
  //calculate pid
  left_motor_vel_pid_output = (left_motor_vel_error_mm_s*VEL_KP) +
                              (left_motor_vel_error_sum_mm_s*VEL_KI) +
                              (left_motor_vel_error_sub_mm_s*VEL_KD);

  right_motor_vel_pid_output = (right_motor_vel_error_mm_s*VEL_KP) +
                              (right_motor_vel_error_sum_mm_s*VEL_KI) +
                              (right_motor_vel_error_sub_mm_s*VEL_KD);    
                              
  //if we don't need any other treadtment on pid_output variables than the variables are fed directly to the motors
  //i guess we need some constraints or regulation on raw output pid values but will ignore for now
  right_motor_cmd = right_motor_vel_pid_output * WHEEL_GAIN;
  left_motor_cmd =  left_motor_vel_pid_output ;                          
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
  servo_angle_pid_output = (orientation_error_deg*STEERING_KP)+
                           (orientation_error_sum_deg*STEERING_KI)+
                           (orientation_error_sub_deg*STEERING_KD);
  
  //servo cmd = pid_output (+ constraint)
  servo_angle_cmd_deg = constrain(servo_angle_pid_output, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
}