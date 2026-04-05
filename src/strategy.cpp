#include "strategy.h"

// ── Public variable definitions ──────────────────────
volatile uint8_t strategy_counter = 0;
volatile uint8_t active_strategy  = 0;
float            strategy_speed   = STRATEGY_0_SPEED;
uint32_t read_ir_start_time = 0; 

// ── Private state for button edge detection ──────────
static bool btn_prev = false;

// ─────────────────────────────────────────────────────
void Strategy_Update() {
    if (strategy_counter == 0) {
        active_strategy = 0;
        strategy_speed  = STRATEGY_0_SPEED;
        read_ir_start_time = IR_START_TIME_FAST_SPEED;
    } else if (strategy_counter == 1) {   // even: 2, 4
        active_strategy = 1;
        strategy_speed  = STRATEGY_1_SPEED;
        read_ir_start_time = IR_START_TIME_MEDIUM_SPEED;
    } else {                                   // 2
        active_strategy = 2;
        strategy_speed  = STRATEGY_2_SPEED;
        read_ir_start_time = IR_START_TIME_SLOW_SPEED;
    }
}

// ─────────────────────────────────────────────────────
void Strategy_Display(Adafruit_SSD1306 &display) {
    display.clearDisplay();

    // ── Row 0: raw counter ───────────────────────────
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(F("CNT: "));
    display.println(strategy_counter);

    // ── Row 1: active strategy ───────────────────────
    display.setCursor(0, 22);
    display.print(F("STR: "));
    display.println(active_strategy);

    // ── Row 2: speed ─────────────────────────────────
    display.setTextSize(1);
    display.setCursor(0, 46);
    display.print(F("SPD: "));
    display.print((int)strategy_speed);
    display.println(F(" mm/s"));

    display.display();
}

// ─────────────────────────────────────────────────────
void Strategy_Init(Adafruit_SSD1306 &display) {
    pinMode(STRATEGY_BTN_PIN, INPUT_PULLUP);
    btn_prev = false;
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
}
