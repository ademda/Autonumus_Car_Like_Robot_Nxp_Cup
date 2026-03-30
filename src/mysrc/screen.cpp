#include "screen.h"

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLE DEFINITIONS
// ─────────────────────────────────────────────────────────────

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

void Screen_Init(TwoWire* wireInterface) {
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(F("HELLO"));
    display.display();
}

void Screen_ShowHello() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(F("HELLO"));
    display.display();
}

void Screen_ShowJackTriggered() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);
    display.println(F("JACK TRIGGERED"));
    display.display();
}

void Screen_TurnOff() {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void Screen_Clear() {
    display.clearDisplay();
}

void Screen_Print(const char* text) {
    display.println(text);
}

void Screen_PrintAt(uint8_t x, uint8_t y, const char* text) {
    display.setCursor(x, y);
    display.println(text);
}

void Screen_Update() {
    display.display();
}
