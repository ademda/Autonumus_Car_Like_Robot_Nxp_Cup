#include "steering.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLE DEFINITIONS
// ─────────────────────────────────────────────────────────────

// Servo command
volatile int16_t servo_angle_cmd_deg = 93;

// Camera input
volatile float last_camera_angle = 87.0;
volatile float last_camera_angle_updated = 87.0;
volatile uint32_t servo_wait = 0;
float filtered_camera_angle = 87.0;

// Steering/Orientation control
volatile float orientation_setpoint_deg = 0;
volatile float orientation_error_deg = 0;
volatile float orientation_error_sum_deg = 0;
volatile float orientation_last_error_deg = 0;
volatile float servo_angle_pid_output = 0;

// PID gains
volatile float steering_kp = STEERING_KP;
volatile float steering_ki = STEERING_KI;
volatile float steering_kd = STEERING_KD;

// Servo instance
Servo steer_servo;

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

void Servo_Init() {
    steer_servo.attach(SERVO_PIN);
    steer_servo.write(SERVO_INIT_ANGLE);
    servo_wait = millis();
}

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
    if (servo_angle >= 87)
        servo_angle = 87 + (servo_angle - 87) * STEERING_KP;
    else
        servo_angle = 87 - (87 - servo_angle) * STEERING_KP;

    servo_angle = 180 - servo_angle;
    servo_angle = constrain(servo_angle, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
    servo_angle_cmd_deg = (int16_t)servo_angle;
    steer_servo.write(servo_angle_cmd_deg);
}

void CalculateOrientationError() {
    // Store prev error
    orientation_last_error_deg = orientation_error_deg;
    // Update error
    extern volatile double curr_orientation_deg;
    orientation_error_deg = orientation_setpoint_deg - curr_orientation_deg;
}

void CalculateSteeringPID() {
    orientation_error_sum_deg = constrain(orientation_error_sum_deg + orientation_error_deg,
                                          -MAX_STEERING_ERROR_SUM, MAX_STEERING_ERROR_SUM);
    float orientation_error_sub_deg = orientation_error_deg - orientation_last_error_deg;
    servo_angle_pid_output = (orientation_error_deg * steering_kp) +
                             (orientation_error_sum_deg * steering_ki) +
                             (orientation_error_sub_deg * steering_kd);

    // servo cmd = pid_output (+ constraint)
    servo_angle_cmd_deg = constrain(servo_angle_pid_output, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
}

void SetCameraAngle(float angle) {
    last_camera_angle = angle;
    last_camera_angle_updated = angle;
}
