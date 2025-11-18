#include <Arduino.h>
#include <Encoder.h>
#include <Servo.h>
#include <TimerOne.h>

#define LEFT_ENC_CH1 5
#define LEFT_ENC_CH2 6

#define RIGHT_ENC_CH1 20
#define RIGHT_ENC_CH2 19

#define RIGHTMOTOR_FWD_PWM 23 //Forward PWM
#define RIGHTMOTOR_BWD_PWM 22 //Backward PWM

#define LEFTMOTOR_FWD_PWM 2 //Forward PWM
#define LEFTMOTOR_BWD_PWM 3 //Backward PWM

#define TIM1_PIN 8
#define TIM3_PIN 9

#define SERVO_PIN 9  

#define SPI_CS_PIN 10   ya turki badel pinet spi ll camera 
#define SPI_MOSI_PIN 11  
#define SPI_MISO_PIN 12
#define SPI_SCLK_PIN 13

volatile int32_t left_ticks_i32, right_ticks_i32;
volatile int32_t right_motor_cmd_i32, left_motor_cmd_i32;
volatile int16_t servo_angle_i16; //in deg

Encoder left_encoder(LEFT_ENC_CH1, LEFT_ENC_CH2);
Encoder right_encoder(RIGHT_ENC_CH1, RIGHT_ENC_CH2);
Servo steer_servo;

void setup() {
  left_encoder.write(0);
  right_encoder.write(0);
  steer_servo.attach(SERVO_PIN);
}

void loop() {
}


void ReadEncoders(){
  left_ticks_i32 = left_encoder.read();
  right_ticks_i32 = right_encoder.read();
} 

void RotateMotors(){
  if (right_motor_cmd_i32>=0){
    analogWrite(RIGHTMOTOR_FWD_PWM, right_motor_cmd_i32);
    analogWrite(RIGHTMOTOR_BWD_PWM, 0);
  }
  else {
    analogWrite(RIGHTMOTOR_FWD_PWM, 0);
    analogWrite(RIGHTMOTOR_BWD_PWM, right_motor_cmd_i32);
  }
  if (left_motor_cmd_i32>=0){
    analogWrite(LEFTMOTOR_FWD_PWM, left_motor_cmd_i32);
    analogWrite(LEFTMOTOR_BWD_PWM, 0);
  }
  else {
    analogWrite(LEFTMOTOR_FWD_PWM, 0);
    analogWrite(LEFTMOTOR_BWD_PWM, left_motor_cmd_i32);
  }
}

void SetServoAngle(){
  steer_servo.write(servo_angle_i16);
}