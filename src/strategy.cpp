#include "strategy.h"

// ── Public variable definitions ──────────────────────
volatile uint8_t strategy_counter = 0;
volatile uint8_t active_strategy  = 0;
float            strategy_speed   = STRATEGY_0_SPEED;
volatile bool    velocity_profile_enabled = false;
volatile bool    infrared_enabled = true;  // IR sensors on by default
uint32_t read_ir_start_time = 0; 

// ── Private state for button edge detection ──────────
static bool btn_prev = false;
static bool ir_btn_prev = false;

// ─────────────────────────────────────────────────────
void Strategy_Update() {
    if (strategy_counter == 0) {
        active_strategy = 0;
        strategy_speed  = STRATEGY_0_SPEED;
        velocity_profile_enabled = true;  // ENABLE velocity profile for strategy 0
        read_ir_start_time = IR_START_TIME_VEL_PROFILE_SPEED;
    } else if (strategy_counter == 1) {
        active_strategy = 1;
        strategy_speed  = STRATEGY_1_SPEED;
        velocity_profile_enabled = false;
        read_ir_start_time = IR_START_TIME_FAST_SPEED;
    } else if (strategy_counter == 2) {
        active_strategy = 2;
        strategy_speed  = STRATEGY_2_SPEED;
        velocity_profile_enabled = false;
        read_ir_start_time = IR_START_TIME_MEDIUM_SPEED;
    } else {  // strategy_counter == 3
        active_strategy = 3;
        strategy_speed  = STRATEGY_3_SPEED;
        velocity_profile_enabled = false;
        read_ir_start_time = IR_START_TIME_SLOW_SPEED;
    }
}

// ─────────────────────────────────────────────────────
void Strategy_Display(Adafruit_SSD1306 &display) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // ── Row 0: Counter ───────────────────────────────
    display.setCursor(0, 0);
    display.print(F("CNT: "));
    display.println(strategy_counter + 1);  // Display 1-4 instead of 0-3

    // ── Row 1: Speed ─────────────────────────────────
    display.setCursor(0, 10);
    display.print(F("SPEED: "));
    if (strategy_counter == 0) {
        display.println(F("1800(VP)"));  // VP = Velocity Profile
    } else if (strategy_counter == 1) {
        display.println(F("1400"));
    } else if (strategy_counter == 2) {
        display.println(F("1200"));
    } else {
        display.println(F("1000"));
    }

    // ── Row 2: Infrared Status ────────────────────────
    display.setCursor(0, 22);
    display.print(F("INFRARED: "));
    display.println(infrared_enabled ? F("ON") : F("OFF"));

    display.display();
}

// ─────────────────────────────────────────────────────
void Strategy_Init(Adafruit_SSD1306 &display) {
    pinMode(STRATEGY_BTN_PIN, INPUT_PULLUP);
    pinMode(IR_BTN_PIN, INPUT_PULLUP);
    btn_prev = false;
    ir_btn_prev = false;
    Strategy_Update();
    Strategy_Display(display);
}

// ─────────────────────────────────────────────────────
void Strategy_Poll(Adafruit_SSD1306 &display) {
    bool btn_now = (digitalRead(STRATEGY_BTN_PIN) == LOW);  // active-low

    if (btn_now && !btn_prev) {                             // rising edge
        strategy_counter = (strategy_counter + 1) % (STRATEGY_MAX_COUNT + 1);
        Strategy_Update();
        Strategy_Display(display);
        delay(30);                                          // debounce
    }

    btn_prev = btn_now;

    // ── Handle IR enable/disable button ──────────────
    bool ir_btn_now = (digitalRead(IR_BTN_PIN) == LOW);  // active-low

    if (ir_btn_now && !ir_btn_prev) {                   // rising edge
        infrared_enabled = !infrared_enabled;            // toggle IR on/off
        Strategy_Display(display);
        delay(30);                                        // debounce
    }

    ir_btn_prev = ir_btn_now;
}
