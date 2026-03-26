#pragma once
#include <Arduino.h>

// ── Pin defines (move these out of main) ─────────────────────
#define IR_1_PIN  15 //15
#define IR_2_PIN  16 //16
#define IR_3_PIN  14  //14
#define IR_4_PIN  21
//theni (ir1)
//raba3 ir4
//thelth (ir2)
//lwl ir3
// ── Behaviour defines ─────────────────────────────────────────
#define IR_BLACK_THRESHOLD   0xFFF   // fallback raw threshold (uncalibrated)
#define IR_READ_INTERVAL_MS  1

// ── Raw readings (same names as before) ──────────────────────
extern int16_t ir1_raw, ir2_raw, ir3_raw, ir4_raw;

// ── Black flags (same names as before) ───────────────────────
extern bool ir1_black, ir2_black, ir3_black, ir4_black;

// ── Calibration min/max (same names as before) ───────────────
extern int16_t ir1_min, ir1_max;
extern int16_t ir2_min, ir2_max;
extern int16_t ir3_min, ir3_max;
extern int16_t ir4_min, ir4_max;
extern bool    ir_calibrated;

// ── Trigger flag (same name as before) ───────────────────────
// One-shot: set once when ≥2 sensors go black.
// Stays true until resetIRStop() is called.
extern volatile bool ir_stop_triggered;

extern uint32_t last_ir_read_ms;

// ── Public API (same names as before + reset for one-shot) ───
void CalibrateIRSensors();
void ReadIRSensors();
void resetIRStop();     // call after handling the stop event