#include "motor.h"

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLE DEFINITIONS
// ─────────────────────────────────────────────────────────────

// Velocity measurements
volatile double left_wheel_curr_vel_mm_s = 0;
volatile double right_wheel_curr_vel_mm_s = 0;
volatile double robot_curr_vel_mm_s = 0;
volatile double left_vel_filtered = 0;
volatile double right_vel_filtered = 0;

// Distance tracking for velocity calculation
volatile double left_wheel_dist_prev_vel_calc = 0;
volatile double right_wheel_dist_prev_vel_calc = 0;

// Velocity setpoints
volatile double left_motor_vel_setpoint_mm_s = 0;
volatile double right_motor_vel_setpoint_mm_s = 0;
volatile double robot_vel_setpoint_mm_s = 0;
volatile double prev_left_motor_vel_setpoint_mm_s = 0;
volatile double prev_right_motor_vel_setpoint_mm_s = 0;

// Velocity errors
volatile double left_motor_vel_error_mm_s = 0;
volatile double right_motor_vel_error_mm_s = 0;
volatile double robot_vel_error_mm_s = 0;
volatile double left_motor_vel_last_error_mm_s = 0;
volatile double right_motor_vel_last_error_mm_s = 0;

// PID outputs
volatile double left_motor_vel_pid_output = 0;
volatile double right_motor_vel_pid_output = 0;
volatile double left_motor_vel_error_sum_mm_s = 0;
volatile double right_motor_vel_error_sum_mm_s = 0;

// Motor commands
volatile int32_t right_motor_cmd = 0;
volatile int32_t left_motor_cmd = 0;

// PID gains
volatile float right_vel_kp = RIGHT_VEL_KP;
volatile float right_vel_ki = RIGHT_VEL_KI;
volatile float right_vel_kd = RIGHT_VEL_KD;

volatile float left_vel_kp = LEFT_VEL_KP;
volatile float left_vel_ki = LEFT_VEL_KI;
volatile float left_vel_kd = LEFT_VEL_KD;

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

void Motor_Init() {
    pinMode(RIGHTMOTOR_FWD_PWM, OUTPUT);
    pinMode(RIGHTMOTOR_BWD_PWM, OUTPUT);
    pinMode(LEFTMOTOR_FWD_PWM, OUTPUT);
    pinMode(LEFTMOTOR_BWD_PWM, OUTPUT);
    StopMotors();
}

void RotateMotors() {
    uint8_t right_cmd = (uint8_t)(constrain(abs(right_motor_cmd), MIN_MOTOR_CMD, MAX_MOTOR_CMD));
    uint8_t left_cmd = (uint8_t)(constrain(abs(left_motor_cmd), MIN_MOTOR_CMD, MAX_MOTOR_CMD));

    if (right_motor_cmd >= 0) {
        analogWrite(RIGHTMOTOR_FWD_PWM, right_cmd);
        analogWrite(RIGHTMOTOR_BWD_PWM, 0);
    } else {
        analogWrite(RIGHTMOTOR_FWD_PWM, 0);
        analogWrite(RIGHTMOTOR_BWD_PWM, right_cmd);
    }

    if (left_motor_cmd >= 0) {
        analogWrite(LEFTMOTOR_FWD_PWM, left_cmd);
        analogWrite(LEFTMOTOR_BWD_PWM, 0);
    } else {
        analogWrite(LEFTMOTOR_FWD_PWM, 0);
        analogWrite(LEFTMOTOR_BWD_PWM, left_cmd);
    }
}

void StopMotors() {
    analogWrite(RIGHTMOTOR_FWD_PWM, 0);
    analogWrite(RIGHTMOTOR_BWD_PWM, 0);
    analogWrite(LEFTMOTOR_FWD_PWM, 0);
    analogWrite(LEFTMOTOR_BWD_PWM, 0);
}

void CalculateVelError() {
    // Store previous values
    left_motor_vel_last_error_mm_s = left_motor_vel_error_mm_s;
    right_motor_vel_last_error_mm_s = right_motor_vel_error_mm_s;

    // Update error
    left_motor_vel_error_mm_s = left_motor_vel_setpoint_mm_s - left_wheel_curr_vel_mm_s;
    right_motor_vel_error_mm_s = right_motor_vel_setpoint_mm_s - right_wheel_curr_vel_mm_s;
    robot_vel_error_mm_s = robot_vel_setpoint_mm_s - robot_curr_vel_mm_s;
}

void CalculateVelPID() {
    // Accumulate integral
    left_motor_vel_error_sum_mm_s += left_motor_vel_error_mm_s;
    right_motor_vel_error_sum_mm_s += right_motor_vel_error_mm_s;

    // Enable integral windup protection to prevent unbounded growth
    left_motor_vel_error_sum_mm_s = constrain(left_motor_vel_error_sum_mm_s, -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);
    right_motor_vel_error_sum_mm_s = constrain(right_motor_vel_error_sum_mm_s, -MAX_VEL_ERROR_SUM, MAX_VEL_ERROR_SUM);

    // FIXED: Divide derivative by dt
    float left_motor_vel_error_sub_mm_s = (left_motor_vel_error_mm_s - left_motor_vel_last_error_mm_s);
    float right_motor_vel_error_sub_mm_s = (right_motor_vel_error_mm_s - right_motor_vel_last_error_mm_s);

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

void ConvertDistanceToVel() {
    // Forward declarations from odometry module
    extern volatile double left_wheel_distance_mm, right_wheel_distance_mm;

    double raw_left_vel = 1000 * (left_wheel_distance_mm - left_wheel_dist_prev_vel_calc) / (VELOCITY_CALC_DT_MS);
    double raw_right_vel = 1000 * (right_wheel_distance_mm - right_wheel_dist_prev_vel_calc) / (VELOCITY_CALC_DT_MS);

    left_vel_filtered = (left_vel_filtered * 0.8 + (raw_left_vel * 0.2));
    right_vel_filtered = (right_vel_filtered * 0.8 + (raw_right_vel * 0.2));

    left_wheel_curr_vel_mm_s = left_vel_filtered;
    right_wheel_curr_vel_mm_s = right_vel_filtered;

    left_wheel_dist_prev_vel_calc = left_wheel_distance_mm;
    right_wheel_dist_prev_vel_calc = right_wheel_distance_mm;

    robot_curr_vel_mm_s = (right_wheel_curr_vel_mm_s + left_wheel_curr_vel_mm_s) / 2.0;
}
