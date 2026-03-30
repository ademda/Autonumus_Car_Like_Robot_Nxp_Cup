#pragma once
#include <Arduino.h>
#include <Servo.h>

// ─────────────────────────────────────────────────────────────
//  SERVO Control Module
//  Manages steering servo and heading PID
// ─────────────────────────────────────────────────────────────

/* Pin Defines */
#define SERVO_PIN 17

/* Servo Angle Limits */
#define MAX_SERVO_ANGLE 135    // 125 //imin //127
#define MIN_SERVO_ANGLE 42     // 47 //55 //isar //47
#define SERVO_INIT_ANGLE 87

/* Steering PID Defines */
#define STEERING_KP 1.0
#define STEERING_KI 0.0
#define STEERING_KD 0.0
#define MAX_STEERING_ERROR_SUM 120  // Prevent integral windup

/* Steering Control */
#define WHEEL_BASE_MM 194  // distance between wheels
#define K_STRAIGHT 2.5    // Gain for small corrections //5.0
#define K_SHARP 6         // Gain for sharp turns
#define GAIN_THRESHOLD 33.0  // Angle (deg) where we start switching to high gain
#define CAMERA_SMOOTHING 0.7 // 0 to 1. Lower is smoother, higher is more responsive.

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLES
// ─────────────────────────────────────────────────────────────

// Servo command and camera input
extern volatile int16_t servo_angle_cmd_deg;
extern volatile float last_camera_angle;
extern volatile float last_camera_angle_updated;
extern volatile uint32_t servo_wait;
extern float filtered_camera_angle;

// Steering/Orientation errors (optional for manual control)
extern volatile float orientation_setpoint_deg, orientation_error_deg, orientation_error_sum_deg;
extern volatile float orientation_last_error_deg;
extern volatile float servo_angle_pid_output;

// PID gains
extern volatile float steering_kp, steering_ki, steering_kd;

// Servo instance
extern Servo steer_servo;

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

/**
 * Initialize servo
 */
void Servo_Init();

/**
 * Set servo angle based on camera input with adaptive steering gain
 * Call periodically, ideally at lower frequency than main loop
 */
void SetServoAngle();

/**
 * Calculate orientation error (for manual steering mode)
 */
void CalculateOrientationError();

/**
 * Calculate steering PID output (for manual steering mode)
 */
void CalculateSteeringPID();

/**
 * Update camera angle from vision system
 */
void SetCameraAngle(float angle);
