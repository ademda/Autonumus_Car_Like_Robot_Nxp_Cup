#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// ── Pin ──────────────────────────────────────────────
#define STRATEGY_BTN_PIN 0   // change to any free Teensy pin

// ── Counter limits ───────────────────────────────────
#define STRATEGY_MAX_COUNT 2  // cycles 0 → 1 → 2  → 0

// ── Speed setpoints per strategy (mm/s) ──────────────
#define STRATEGY_0_SPEED 1400.0f
#define STRATEGY_1_SPEED 1200.0f
#define STRATEGY_2_SPEED 1000.0f

// ── Public variables ─────────────────────────────────
extern volatile uint8_t strategy_counter;  // raw counter  0..4
extern volatile uint8_t active_strategy;   // derived: 0, 1 or 2
extern float            strategy_speed;    // speed for the active strategy

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
 * Derive active_strategy (0/1/2) and strategy_speed
 * from strategy_counter.
 *   counter == 0        → strategy 0  (speed 1400)
 *   counter even (2,4)  → strategy 1  (speed 1200)
 *   counter odd  (1,3)  → strategy 2  (speed 1000)
 */
void Strategy_Update();

/**
 * Refresh the OLED with current counter / strategy / speed.
 */
void Strategy_Display(Adafruit_SSD1306 &display);
