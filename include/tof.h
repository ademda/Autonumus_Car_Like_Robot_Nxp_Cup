#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ToFFilter.h>

// ─────────────────────────────────────────────────────────────
//  TOF (Time of Flight) Sensor Module
//  Manages VL53L0X distance sensors
// ─────────────────────────────────────────────────────────────

/* ToF addresses */
#define TOF_ADDR_1 0x30

/* ToF Configuration */
#define TOF_INTERVAL_MS 100       // ~10 Hz reading
#define STOP_DISTANCE 400         // mm, distance at which we consider the robot has reached the target

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLES (extern in tof.h)
// ─────────────────────────────────────────────────────────────

extern float left_tof_distance;
extern float center_tof_distance;
extern float right_tof_distance;
extern bool cube_detected;
extern VL53L0X_RangingMeasurementData_t tof_measure;
extern ToFFilter tofFilter;
extern Adafruit_VL53L0X tof1;
extern uint32_t last_tof_test;

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

/**
 * Initialize ToF sensor
 * Call with the Wire object for I2C communication
 */
void ToF_Init(TwoWire* wireInterface, uint8_t sda_pin, uint8_t scl_pin);

/**
 * Read ToF sensors non-blocking
 * Call periodically from main loop
 */
void readToFsNonBlocking();

/**
 * Reset cube detection flag
 */
void resetCubeDetected();
