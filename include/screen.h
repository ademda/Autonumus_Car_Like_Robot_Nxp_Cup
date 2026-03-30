#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

// ─────────────────────────────────────────────────────────────
//  SCREEN/DISPLAY Module
//  Manages SSD1306 OLED display
// ─────────────────────────────────────────────────────────────

/* Screen Configuration */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLES
// ─────────────────────────────────────────────────────────────

extern Adafruit_SSD1306 display;

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

/**
 * Initialize the SSD1306 display
 * Must be called before any display operations
 */
void Screen_Init(TwoWire* wireInterface);

/**
 * Clear display and show hello message
 */
void Screen_ShowHello();

/**
 * Display jack triggered message
 */
void Screen_ShowJackTriggered();

/**
 * Turn display off
 */
void Screen_TurnOff();

/**
 * Clear display
 */
void Screen_Clear();

/**
 * Display text on screen
 */
void Screen_Print(const char* text);

/**
 * Display text at specific position
 */
void Screen_PrintAt(uint8_t x, uint8_t y, const char* text);

/**
 * Update display
 */
void Screen_Update();
