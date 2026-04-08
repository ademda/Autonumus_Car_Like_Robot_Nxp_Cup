#include "infrared.h"
#include <math.h>
#include <Adafruit_SSD1306.h>

// Forward declaration for display callback (defined in main.cpp)
extern Adafruit_SSD1306 display;

// ─────────────────────────────────────────────────────────────
//  INTERNAL ADAPTIVE FILTER
//  Pipeline per sensor:
//    Raw ADC → Median(3) spike rejection
//            → Sigmoid-alpha EMA  (fast on large Δ, slow on noise)
//            → Two-phase calibration (white EMA, black EMA)
//            → Normalised 0.0 (white) … 1.0 (black)
// ─────────────────────────────────────────────────────────────

// ── Tuning ───────────────────────────────────────────────────
#define CALIB_TOTAL_MS    10000U   // total calibration window (ms): 5s white + 5s black
#define CALIB_WHITE_MS     5000U   // first phase: sensors on white (static 5 seconds)
#define CALIB_BLACK_MS     5000U   // second phase: sensors on black (static 5 seconds)
#define ALPHA_MIN_F        0.04f   // EMA: slowest (tiny delta)
#define ALPHA_MAX_F        0.92f   // EMA: fastest (large step)
#define SIGMOID_SCALE_F    3.0f    // sharpness of alpha transition
#define SIGMOID_BIAS_F     1.5f    // midpoint of alpha transition
#define DELTA_NORM_F       200.0f  // delta that gives ~mid alpha
#define MIN_SPAN           150     // minimum white→black spread
#define NORM_THRESHOLD     0.3f   // normalised cutoff → black=true
#define MIN_BLACK_SENSORS  2       // sensors needed to latch stop


// ── Per-sensor state (private) ────────────────────────────────w
typedef struct {
    // Median(3) buffer
    int16_t  medBuf[3];
    uint8_t  medIdx;
    bool     medFilled;
    // EMA
    float    ema;           // -1 = uninitialised
    // Calibration
    float    whiteEMA;      // -1 = not yet learned
    float    blackEMA;      // -1 = not yet learned
} IRSensor_t;

static IRSensor_t _sensors[4];  // indexed 0..3 = IR1..IR4
int16_t threshold1;
int16_t threshold2;
int16_t threshold3;
int16_t threshold4;

// ── Helpers (file-scope only) ─────────────────────────────────

static void _pushMedian(IRSensor_t *s, int16_t v) {
    s->medBuf[s->medIdx] = v;
    s->medIdx = (s->medIdx + 1) % 3;
    if (s->medIdx == 0) s->medFilled = true;
}

static int16_t _getMedian(const IRSensor_t *s) {
    if (!s->medFilled) return s->medBuf[(s->medIdx + 2) % 3];
    int16_t a = s->medBuf[0], b = s->medBuf[1], c = s->medBuf[2];
    int16_t t;
    if (a > b) { t=a; a=b; b=t; }
    if (b > c) { t=b; b=c; c=t; }
    if (a > b) { t=a; a=b; b=t; }
    return b;
}

// Feed a raw sample through median + sigmoid EMA; returns filtered value
static float _updateEMA(IRSensor_t *s, int16_t raw) {
    _pushMedian(s, raw);
    int16_t med = _getMedian(s);

    if (s->ema < 0.0f) {
        s->ema = (float)med;
        return s->ema;
    }

    float diff        = (float)med - s->ema;
    float delta       = fabsf(diff);
    float deltaFactor = delta / DELTA_NORM_F;
    if (deltaFactor > 2.0f) deltaFactor = 2.0f;

    // Sigmoid: small delta → alpha≈ALPHA_MIN, large delta → alpha≈ALPHA_MAX
    float curve = 1.0f / (1.0f + expf(-(deltaFactor * SIGMOID_SCALE_F - SIGMOID_BIAS_F)));
    float alpha = ALPHA_MIN_F + (ALPHA_MAX_F - ALPHA_MIN_F) * curve;

    s->ema += alpha * diff;
    return s->ema;
}

static void _learnWhite(IRSensor_t *s) {
    if (s->ema < 0.0f) return;
    s->whiteEMA = (s->whiteEMA < 0.0f) ? s->ema
                                        : s->whiteEMA * 0.85f + s->ema * 0.15f;
}

static void _learnBlack(IRSensor_t *s) {
    if (s->ema < 0.0f) return;
    if (s->blackEMA < 0.0f)   { s->blackEMA = s->ema; return; }
    if (s->ema > s->blackEMA)    s->blackEMA = s->blackEMA * 0.7f + s->ema * 0.3f;
}

static void _finalise(IRSensor_t *s, int16_t *minOut, int16_t *maxOut) {
    if (s->whiteEMA < 0.0f || s->blackEMA < 0.0f) return;
    if ((s->blackEMA - s->whiteEMA) < (float)MIN_SPAN)
        s->blackEMA = s->whiteEMA + (float)MIN_SPAN;
    // store into legacy min/max variables for debug / external visibility
    *minOut = (int16_t)s->whiteEMA;
    *maxOut = (int16_t)s->blackEMA;
}

// Normalised value 0.0→1.0; uses calibrated min/max
static float _normalised(const IRSensor_t *s) {
    if (s->whiteEMA < 0.0f || s->blackEMA <= s->whiteEMA) return 0.0f;
    float n = (s->ema - s->whiteEMA) / (s->blackEMA - s->whiteEMA);
    if (n < 0.0f) n = 0.0f;
    if (n > 1.0f) n = 1.0f;
    return n;
}

static void _initSensor(IRSensor_t *s) {
    s->medBuf[0] = s->medBuf[1] = s->medBuf[2] = 0;
    s->medIdx    = 0;
    s->medFilled = false;
    s->ema       = -1.0f;
    s->whiteEMA  = -1.0f;
    s->blackEMA  = -1.0f;
}


// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLE DEFINITIONS  (declared extern in infrared.h)
// ─────────────────────────────────────────────────────────────

int16_t ir1_raw = 0, ir2_raw = 0, ir3_raw = 0, ir4_raw = 0;

bool ir1_black = false, ir2_black = false;
bool ir3_black = false, ir4_black = false;

int16_t ir1_min = 32767, ir1_max = -32768;
int16_t ir2_min = 32767, ir2_max = -32768;
int16_t ir3_min = 32767, ir3_max = -32768;
int16_t ir4_min = 32767, ir4_max = -32768;

bool ir_calibrated = false;

volatile bool ir_stop_triggered = false;

uint32_t last_ir_read_ms = 0;


// ─────────────────────────────────────────────────────────────
//  CalibrateIRSensors()
//  Minimal two-phase calibration:
//  Phase 1 (5000ms): sensors on white → track minimum raw values
//  Phase 2 (5000ms): sensors on black → track maximum raw values
//  No filtering, just simple min/max of raw ADC readings.
// ─────────────────────────────────────────────────────────────
void CalibrateIRSensors() {
    // Pin order matches original ReadIRSensors() swap: ch1=IR1, ch2=IR3, ch3=IR2, ch4=IR4
    const uint8_t pins[4] = { IR_1_PIN, IR_3_PIN, IR_2_PIN, IR_4_PIN };
    
    // Sum accumulators
    uint32_t white_sum[4] = {0,0,0,0};
    uint32_t black_sum[4] = {0,0,0,0};
    uint16_t white_count = 0;
    uint16_t black_count = 0;

    Serial.println(F("\n--- IR Calibration (5s White + 5s Black) ---"));
    Serial.println(F("Phase 1: Place sensors on WHITE surface..."));

    uint32_t t0 = millis();
    uint32_t lastDisplayUpdate = 0;

    // Display white phase message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("IR CALIBRATION"));
    display.println(F("---------------"));
    display.println(F("Phase 1/2:"));
    display.println(F("CALIBRATING WHITE"));
    display.println(F("Time: 0/5 sec"));
    display.display();

    // ─────── PHASE 1: WHITE ───────
    while (millis() - t0 < CALIB_WHITE_MS) {
        int16_t raw[4];
        for (uint8_t i = 0; i < 4; i++) {
            raw[i] = (int16_t)analogRead(pins[i]);
            white_sum[i] += raw[i];
        }
        white_count++;

        // Update display every 1 second
        if (millis() - lastDisplayUpdate > 1000) {
            uint32_t elapsed = (millis() - t0) / 1000;
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println(F("IR CALIBRATION"));
            display.println(F("Phase 1/2: WHITE"));
            display.print(F("Time: "));
            display.print(elapsed);
            display.println(F("/5 sec"));
            display.display();
            lastDisplayUpdate = millis();
        }

        delay(10);  // 100 Hz sampling
    }

    Serial.println(F("Phase 1 Complete. Now place sensors on BLACK surface you have 4 seconds..."));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Phase 1 Complete."));
    display.println(F("Put on BLACK surface"));
    display.println(F("Wait 4 sec..."));
    display.display();
    delay(4000);

    // Phase 2: Black
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("IR CALIBRATION"));
    display.println(F("Phase 2/2: BLACK"));
    display.println(F("Time: 0/5 sec"));
    display.display();

    lastDisplayUpdate = 0;
    uint32_t phase2Start = millis();

    while (millis() - phase2Start < CALIB_BLACK_MS) {
        int16_t raw[4];
        for (uint8_t i = 0; i < 4; i++) {
            raw[i] = (int16_t)analogRead(pins[i]);
            black_sum[i] += raw[i];
        }
        black_count++;

        // Update display every 1 second
        if (millis() - lastDisplayUpdate > 1000) {
            uint32_t elapsed = (millis() - phase2Start) / 1000;
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0, 0);
            display.println(F("IR CALIBRATION"));
            display.println(F("Phase 2/2: BLACK"));
            display.print(F("Time: "));
            display.print(elapsed);
            display.println(F("/5 sec"));
            display.display();
            lastDisplayUpdate = millis();
        }

        delay(10);  // 100 Hz sampling
    }

    // Compute averages and thresholds
    float avg_white[4], avg_black[4], threshold[4];
    for (uint8_t i = 0; i < 4; i++) {
        avg_white[i] = (float)white_sum[i] / white_count;
        avg_black[i] = (float)black_sum[i] / black_count;
        threshold[i] = (avg_white[i] + avg_black[i]) * 0.5;
    }

    // Store values
    ir1_min = avg_white[0]; ir1_max = avg_black[0]; threshold1 = threshold[0];
    ir2_min = avg_white[1]; ir2_max = avg_black[1]; threshold2 = threshold[1];
    ir3_min = avg_white[2]; ir3_max = avg_black[2]; threshold3 = threshold[2];
    ir4_min = avg_white[3]; ir4_max = avg_black[3]; threshold4 = threshold[3];

    ir_calibrated = true;

    // Display completion message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("IR CALIBRATION"));
    display.println(F("COMPLETE!"));
    for (uint8_t i = 0; i < 4; i++) {
        display.print(F("IR")); display.print(i+1);
        display.print(F(": W=")); display.print(avg_white[i],1);
        display.print(F(" B=")); display.println(avg_black[i],1);
        // display.print(F(" T=")); display.println(threshold[i],1);
    }
    display.display();
    delay(2000);

    // Serial output
    // Serial.println(F("Calibration done (avg white / avg black / threshold):"));
    for (uint8_t i = 0; i < 4; i++) {
        Serial.print(F("IR")); Serial.print(i+1);
        Serial.print(F(": W=")); Serial.print(avg_white[i],1);
        Serial.print(F(" B=")); Serial.print(avg_black[i],1);
        Serial.print(F(" T=")); Serial.println(threshold[i],1);
    }
}


// ─────────────────────────────────────────────────────────────
//  ReadIRSensors()
//  Simple threshold-based detection using calibrated min/max.
// ─────────────────────────────────────────────────────────────
void ReadIRSensors() {
    last_ir_read_ms = millis();
    uint32_t current_time = millis();

    // Read raw ADC values (pin swap as per original)
    ir1_raw = (int16_t)analogRead(IR_1_PIN);
    ir2_raw = (int16_t)analogRead(IR_3_PIN);
    ir3_raw = (int16_t)analogRead(IR_2_PIN);
    ir4_raw = (int16_t)analogRead(IR_4_PIN);
    

    if (ir_calibrated) {
        // Use simple midpoint threshold between white and black

        ir1_black = (ir1_raw > threshold1);
        ir2_black = (ir2_raw > threshold2);
        ir3_black = (ir3_raw > threshold3);
        ir4_black = (ir4_raw > threshold4);
    }

    // Count currently detected black sensors
    uint8_t blackCount = 0;
    if (ir1_black) blackCount++;
    if (ir2_black) blackCount++;
    if (ir3_black) blackCount++;
    if (ir4_black) blackCount++;

    // One-shot latch — fires once if more than one sensor currently detecting black
    if (!ir_stop_triggered && blackCount >= MIN_BLACK_SENSORS) {
        ir_stop_triggered = true;
    }
}


// ─────────────────────────────────────────────────────────────
//  resetIRStop()
//  Call after the robot has acted on ir_stop_triggered
//  to allow the next detection event to latch again.
// ─────────────────────────────────────────────────────────────
void resetIRStop() {
    ir_stop_triggered = false;
}