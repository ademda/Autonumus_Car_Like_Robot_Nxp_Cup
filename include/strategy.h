#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// ── Pin ──────────────────────────────────────────────
#define STRATEGY_BTN_PIN 0   // change to any free Teensy pin

// ── Counter limits ───────────────────────────────────
#define STRATEGY_MAX_COUNT 3  // cycles 0 → 1 → 2 → 3 → 0

// ── Speed setpoints per strategy (mm/s) ──────────────
#define STRATEGY_0_SPEED 1800.0f  // strategy 0 uses velocity profile
#define STRATEGY_1_SPEED 1400.0f
#define STRATEGY_2_SPEED 1200.0f
#define STRATEGY_3_SPEED 1000.0f

// ── Velocity Profile Parameters (for strategy 3) ─────
#define VELOCITY_PROFILE_MAX_SPEED 1800.0f   // max speed on straight
#define VELOCITY_PROFILE_MIN_SPEED 1200.0f   // min speed in sharp turn
/***************** IR SENSOR TIMING DEFINES (Speed-based) *********** */
#define IR_START_TIME_SLOW_SPEED 5000//13000    //ms - time when IR sensors start acquiring data (slow speed)
#define IR_START_TIME_MEDIUM_SPEED 5000//10000  //ms - time when IR sensors start acquiring data (medium speed)
#define IR_START_TIME_FAST_SPEED 4000//8000    //ms - time when IR sensors start acquiring data (fast speed)
#define IR_START_TIME_VEL_PROFILE_SPEED 4000//8000
// ── Public variables ─────────────────────────────────
extern volatile uint8_t strategy_counter;  // raw counter  0..4
extern volatile uint8_t active_strategy;   // derived: 0, 1, 2, or 3
extern float            strategy_speed;    // speed for the active strategy
extern volatile bool    velocity_profile_enabled;  // true if strategy 3 is active

// ── Functions ────────────────────────────────────────

/**
 * Call once in setup() BEFORE the jack-wait loop.
 * Configures STRATEGY_BTN_PIN and shows the initial screen.
 */
void Strategy_Init(Adafruit_SSD1306 &display);

/**
 * Call repeatedly INSIDE the jack-wait while() loop.
 * Detects button presses, updates counter & display.
 */
void Strategy_Poll(Adafruit_SSD1306 &display);

/**
 * Derive active_strategy (0/1/2/3) and strategy_speed
 * from strategy_counter.
 *   counter == 0        → strategy 0  (velocity profile ENABLED with max/min speeds)
 *   counter == 1        → strategy 1  (speed 1400, no velocity profile)
 *   counter == 2        → strategy 2  (speed 1200, no velocity profile)
 *   counter == 3        → strategy 3  (speed 1000, no velocity profile)
 * 
 * velocity_profile_enabled is set to true only for strategy 0.
 */
void Strategy_Update();

/**
 * Refresh the OLED with current counter / strategy / speed.
 */
void Strategy_Display(Adafruit_SSD1306 &display);
