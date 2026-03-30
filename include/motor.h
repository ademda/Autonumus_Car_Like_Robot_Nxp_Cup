#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────
//  MOTOR & PID Control Module
//  Manages motor control and velocity PID loops
// ─────────────────────────────────────────────────────────────

/* Pin Defines */
#define RIGHTMOTOR_FWD_PWM 23  // Forward PWM
#define RIGHTMOTOR_BWD_PWM 22  // Backward PWM
#define LEFTMOTOR_FWD_PWM 6    // Forward PWM
#define LEFTMOTOR_BWD_PWM 7    // Backward PWM

/* Motor Control Limits */
#define MAX_MOTOR_CMD 255
#define MIN_MOTOR_CMD 0

/* PID Defines */
#define RIGHT_VEL_KP 0.25  // 0.1
#define RIGHT_VEL_KI 0.007 // 0.001
#define RIGHT_VEL_KD 0.0   // 0.0

#define LEFT_VEL_KP 0.025
#define LEFT_VEL_KI 0.007
#define LEFT_VEL_KD 0.0

#define MAX_VEL_ERROR_SUM 35000

/* Timing */
#define CONTROL_LOOP_DT_MS 5 // 5ms = 0.005 seconds (200Hz control loop)
#define VELOCITY_CALC_DT_MS 5

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLES
// ─────────────────────────────────────────────────────────────

// Velocity measurements
extern volatile double left_wheel_curr_vel_mm_s, right_wheel_curr_vel_mm_s, robot_curr_vel_mm_s;
extern volatile double left_vel_filtered, right_vel_filtered;

// Velocity setpoints
extern volatile double left_motor_vel_setpoint_mm_s, right_motor_vel_setpoint_mm_s, robot_vel_setpoint_mm_s;

// Velocity errors
extern volatile double left_motor_vel_error_mm_s, right_motor_vel_error_mm_s, robot_vel_error_mm_s;
extern volatile double left_motor_vel_last_error_mm_s, right_motor_vel_last_error_mm_s;

// PID outputs
extern volatile double left_motor_vel_pid_output, right_motor_vel_pid_output;
extern volatile double left_motor_vel_error_sum_mm_s, right_motor_vel_error_sum_mm_s;

// Motor commands
extern volatile int32_t right_motor_cmd, left_motor_cmd;

// PID gains
extern volatile float right_vel_kp, right_vel_ki, right_vel_kd;
extern volatile float left_vel_kp, left_vel_ki, left_vel_kd;

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

/**
 * Initialize motor pins
 */
void Motor_Init();

/**
 * Rotate motors based on motor_cmd values
 * Called from control loop
 */
void RotateMotors();

/**
 * Stop both motors
 */
void StopMotors();

/**
 * Calculate velocity error for both wheels
 */
void CalculateVelError();

/**
 * Calculate velocity PID output
 */
void CalculateVelPID();

/**
 * Convert distance to velocity
 * Call after updating odometry
 */
void ConvertDistanceToVel();
