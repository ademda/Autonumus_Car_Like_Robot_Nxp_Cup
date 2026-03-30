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
#include "tof.h"
#include "motor.h"
#include "steering.h"
#include "screen.h"
//#define DEBUG 1

#define UART_TX 1
#define UART_RX 0

#define LEFT_ENC_CH1 3
#define LEFT_ENC_CH2 2

#define RIGHT_ENC_CH1 5
#define RIGHT_ENC_CH2 4

#define TIM1_PIN 8
#define TIM3_PIN 9

#define JACK_PIN 9     // Jack button (INPUT_PULLUP)

#define SPI_CS_PIN 10   //ya turki badel pinet spi ll camera teensy 4.0
#define SPI_MOSI_PIN 11  
#define SPI_MISO_PIN 12
#define SPI_SCLK_PIN 13

#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19

#define WHEEL_GAIN  1.000

/****************  ODOMETRY DEFINES *********** */
#define LEFT_ENCODER_CPR 408
#define RIGHT_ENCODER_CPR 408
#define LEFT_WHEEL_DIAMETER_MM 67.58 //arbitrary number //65 //91.77 //72.19
#define RIGHT_WHEEL_DIAMETER_MM 67.58 //65 //99.32
#define WHEEL_BASE_MM 194 //distance between wheels
#define SERVO_INIT_ANGLE 87 //87

/********************** ODOMETRY VARIABLES ******************* */
volatile double prev_left_wheel_dist_mm, prev_right_wheel_dist_mm, prev_robot_dist_mm;
volatile double left_wheel_distance_mm, right_wheel_distance_mm, robot_distance_mm;
volatile double curr_orientation_deg;
volatile uint32_t last_vel_calc_ms = 0;

/**************** CONTROL VARIABLES ***** */
//DISTANCE  CONTROL
volatile float distance_setpoint_mm, prev_robot_distance_mm;
volatile float distance_error_mm;
volatile bool distance_control_enable = false;
volatile bool distance_reached = false;
//EMERGENCY STOP
volatile bool emergency_stop_enable = false;
/********* INSTANCES ****** */
QuadEncoder left_encoder(1, LEFT_ENC_CH1, LEFT_ENC_CH2);
QuadEncoder right_encoder(2, RIGHT_ENC_CH1, RIGHT_ENC_CH2);
Vision vision;
/*********** DEBUG VARIABLES **** */
uint32_t last_debug = 0;
uint32_t start_time = 0;

/**************** SENSOR VARIABLES *********** */
volatile int32_t left_ticks_i32, right_ticks_i32;

/************** FUNCTIONS DECLARATIONS  ***** */
void ReadEncoders();
void ConvertTicksToDistance();

void VelControllerRoutine();
void GetOrientation(); // using encoders for now until camera code comes
void VelOdomRoutine();

void CalculateDistanceError();//used for tuning only

void EmptyFunction(){
  
}

void NavRoutine(){
  VelOdomRoutine();
  //GetOrientation();
  VelControllerRoutine(); 
  if (cube_detected == true){
        StopMotors();
  }else {
    if (ir_stop_triggered == true ){
      left_motor_vel_setpoint_mm_s = 750; //750
      right_motor_vel_setpoint_mm_s = 750; //750
    }
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

void setup() {
  /*********** JACK BUTTON INIT ***********/
  pinMode(JACK_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  /*********** VISION & SENSORS INIT ***********/
  vision.begin();
  // Initialize left encoder
  delay(3000);
  vision.pixy.setLamp(1, 1);
  // Set pinMode for IR sensor analog pins (A0, A1, A2, A3)
  pinMode(IR_1_PIN, INPUT);
  pinMode(IR_2_PIN, INPUT);
  pinMode(IR_3_PIN, INPUT);
  pinMode(IR_4_PIN, INPUT);

  Screen_Init(&Wire);

  /**************** SENSOR CALIBRATION ********* */
  CalibrateIRSensors();
  /****************  MOTOR INIT *********** */
  Motor_Init();
  /****************  ENCODERS INIT *********** */
  left_encoder.setInitConfig();
  left_encoder.init();

  // Initialize right encoder
  right_encoder.setInitConfig();
  right_encoder.init();

  left_encoder.write(0);
  right_encoder.write(0);

  /****************  SERVO INIT *********** */
  Servo_Init();
  Serial1.begin(115200);

  //  /*ToF Init */
  ToF_Init(&Wire, I2C_SDA_PIN, I2C_SCL_PIN);
  
  /**************** WAIT FOR JACK **********/
  uint32_t debounce_time = millis();
  Strategy_Init(display);
  while(digitalRead(JACK_PIN) == LOW) {
    if(millis() - debounce_time > 50) {
      Strategy_Poll(display);
      delay(10);

    }
  }
  Screen_ShowJackTriggered();
  delay(100);
  delay(2000); // Debounce delay
  Screen_TurnOff();
  // Display white phase messages
  /**************** TIMERS INIT ********* */
  Timer1.initialize(CONTROL_LOOP_DT_MS*1000);          // set period in µs //5000
  Timer1.attachInterrupt(NavRoutine);  // attach the interrupt function
  //left_motor_vel_setpoint_mm_s = 1200; //750
  //right_motor_vel_setpoint_mm_s = 1200; //750
  left_motor_vel_setpoint_mm_s  = strategy_speed;
  right_motor_vel_setpoint_mm_s = strategy_speed;
  start_time = millis();
}

void loop() {
  uint32_t current_time = millis();
  uint32_t time_diff = current_time - start_time;
  if (time_diff>3000){
    if (!ir_stop_triggered){
      //vision.pixy.setLamp(0, 0);
      ReadIRSensors();
    }
    else {
      readToFsNonBlocking();
      //vision.pixy.setLamp(1, 0);
    }
  }
  String mode;
  float distance;
  float camera_angle = vision.calculate_steering_angle(mode, distance);
  SetCameraAngle(180 - camera_angle); //adem ll test 
  //Serial.print("last_camera_angle");
  //Serial.println(last_camera_angle);
  //Serial.print(">camera_angle:");
  //Serial.println(last_camera_angle_updated);
}

/****************  BASIC FUNCTIONS ********* */
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


void CalculateDistanceError(){//used for tuning only
  distance_error_mm = distance_setpoint_mm - robot_distance_mm; 
}

/****************  ODOMETRY FUNCTIONS ********* */

void ConvertTicksToDistance(){
  left_wheel_distance_mm = (left_ticks_i32 * M_PI * LEFT_WHEEL_DIAMETER_MM) / LEFT_ENCODER_CPR;
  right_wheel_distance_mm = (right_ticks_i32 * M_PI * RIGHT_WHEEL_DIAMETER_MM) / RIGHT_ENCODER_CPR ;
  robot_distance_mm = (left_wheel_distance_mm + right_wheel_distance_mm)/2.0;
}