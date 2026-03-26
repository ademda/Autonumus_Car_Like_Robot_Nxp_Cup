#include "infrared.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────
//  INTERNAL ADAPTIVE FILTER
//  Pipeline per sensor:
//    Raw ADC → Median(3) spike rejection
//            → Sigmoid-alpha EMA  (fast on large Δ, slow on noise)
//            → Two-phase calibration (white EMA, black EMA)
//            → Normalised 0.0 (white) … 1.0 (black)
// ─────────────────────────────────────────────────────────────

// ── Tuning ───────────────────────────────────────────────────
#define CALIB_TOTAL_MS    10000U   // total calibration window (ms)
#define CALIB_WHITE_MS     4000U   // first phase: sensors on white
#define ALPHA_MIN_F        0.04f   // EMA: slowest (tiny delta)
#define ALPHA_MAX_F        0.92f   // EMA: fastest (large step)
#define SIGMOID_SCALE_F    3.0f    // sharpness of alpha transition
#define SIGMOID_BIAS_F     1.5f    // midpoint of alpha transition
#define DELTA_NORM_F       200.0f  // delta that gives ~mid alpha
#define MIN_SPAN           150     // minimum white→black spread
#define NORM_THRESHOLD     0.3f   // normalised cutoff → black=true
#define MIN_BLACK_SENSORS  2       // sensors needed to latch stop


// ── Per-sensor state (private) ────────────────────────────────
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
//  Two-phase adaptive calibration (same name as original).
//  Phase 1 (CALIB_WHITE_MS): sensors on white → learns whiteEMA
//  Phase 2 (remaining):      sensors on black → locks blackEMA
//  All learning uses the filtered signal, not raw ADC.
// ─────────────────────────────────────────────────────────────
void CalibrateIRSensors() {
    for (uint8_t i = 0; i < 4; i++) _initSensor(&_sensors[i]);

    // Pin order matches original ReadIRSensors() swap: ch1=IR1, ch2=IR3, ch3=IR2, ch4=IR4
    const uint8_t pins[4] = { IR_1_PIN, IR_3_PIN, IR_2_PIN, IR_4_PIN };

    Serial.println(F("\n--- IR Calibration (adaptive single phase) ---"));
    Serial.println(F("Keep sensors over the surface for 10 seconds..."));

    uint32_t t0        = millis();
    uint32_t lastPrint = 0;

    while (millis() - t0 < CALIB_TOTAL_MS) {
        // Read + filter all four sensors
        int16_t raw[4];
        for (uint8_t i = 0; i < 4; i++) raw[i] = (int16_t)analogRead(pins[i]);

        for (uint8_t i = 0; i < 4; i++) {
            float val = _updateEMA(&_sensors[i], raw[i]);
            // Continuous min/max tracking
            if (_sensors[i].whiteEMA < 0.0f || val < _sensors[i].whiteEMA) _sensors[i].whiteEMA = val;
            if (_sensors[i].blackEMA < 0.0f || val > _sensors[i].blackEMA) _sensors[i].blackEMA = val;
        }

        // Print progress every 500 ms
        if (millis() - lastPrint > 500) {
            Serial.print(F("raw: "));
            for (uint8_t i = 0; i < 4; i++) Serial.print(raw[i]), Serial.print(' ');
            Serial.print(F("  ema: "));
            for (uint8_t i = 0; i < 4; i++) Serial.print(_sensors[i].ema, 0), Serial.print(' ');
            Serial.println();
            lastPrint = millis();
        }

        delay(4); // ~250 Hz
    }

    // Finalize: ensure minimum span and store in legacy min/max variables
    _finalise(&_sensors[0], &ir1_min, &ir1_max);
    _finalise(&_sensors[1], &ir2_min, &ir2_max);
    _finalise(&_sensors[2], &ir3_min, &ir3_max);
    _finalise(&_sensors[3], &ir4_min, &ir4_max);

    ir_calibrated = true;

    Serial.println(F("Calibration done:"));
    Serial.print(F("IR1 min:")); Serial.print(ir1_min); Serial.print(F(" max:")); Serial.println(ir1_max);
    Serial.print(F("IR2 min:")); Serial.print(ir2_min); Serial.print(F(" max:")); Serial.println(ir2_max);
    Serial.print(F("IR3 min:")); Serial.print(ir3_min); Serial.print(F(" max:")); Serial.println(ir3_max);
    Serial.print(F("IR4 min:")); Serial.print(ir4_min); Serial.print(F(" max:")); Serial.println(ir4_max);
}

// ─────────────────────────────────────────────────────────────
//  ReadIRSensors()
//  Same name as original. Call in loop().
//  Updates ir1_raw…ir4_raw, ir1_black…ir4_black.
//  One-shot latch: ir_stop_triggered goes true once when
//  ≥ MIN_BLACK_SENSORS sensors are black; stays true until
//  resetIRStop() is called.
// ─────────────────────────────────────────────────────────────
void ReadIRSensors() {
    last_ir_read_ms = millis();

    // Same pin swap as original: ch2=IR_3, ch3=IR_2
    ir1_raw = (int16_t)analogRead(IR_1_PIN);
    ir2_raw = (int16_t)analogRead(IR_3_PIN);
    ir3_raw = (int16_t)analogRead(IR_2_PIN);
    ir4_raw = (int16_t)analogRead(IR_4_PIN);

    // Update filter for each sensor
    _updateEMA(&_sensors[0], ir1_raw);
    _updateEMA(&_sensors[1], ir2_raw);
    _updateEMA(&_sensors[2], ir3_raw);
    _updateEMA(&_sensors[3], ir4_raw);
    float n1 ;
    float n2 ;
    float n3;
    float n4 ;

    if (ir_calibrated) {
         n1 = _normalised(&_sensors[0]);
         n2 = _normalised(&_sensors[1]);
         n3 = _normalised(&_sensors[2]);
         n4 = _normalised(&_sensors[3]);

        // Same per-sensor thresholds as original
        ir1_black = (n1 > 0.25f);
        ir2_black = (n2 > 0.25f);
        ir3_black = (n3 > 0.25f);
        ir4_black = (n4 > 0.25f);

    } else {
        // Fallback: raw threshold (uncalibrated), same as original
        ir1_black = (ir1_raw > IR_BLACK_THRESHOLD);
        ir2_black = (ir2_raw > IR_BLACK_THRESHOLD);
        ir3_black = (ir3_raw > IR_BLACK_THRESHOLD);
        ir4_black = (ir4_raw > IR_BLACK_THRESHOLD);
    }
    Serial.print(F("IR black: ")); Serial.print(ir1_black); Serial.print(' '); Serial.print(ir2_black); Serial.print(' '); Serial.print(ir3_black); Serial.print(' '); Serial.println(ir4_black);   
    Serial.print(F("IR norm: ")); Serial.print(n1); Serial.print(' '); Serial.print(n2); Serial.print(' '); Serial.print(n3); Serial.print(' '); Serial.println(n4);        
    //Serial.print(F("IR raw: ")); Serial.print(ir1_raw); Serial.print(' '); Serial.print(ir2_raw); Serial.print(' '); Serial.print(ir3_raw); Serial.print(' '); Serial.println(ir4_raw);
    // One-shot latch — fires exactly once per event, no re-trigger
    uint8_t blackCount = (uint8_t)ir1_black + ir2_black + ir3_black + ir4_black;
    if (!ir_stop_triggered && blackCount >= MIN_BLACK_SENSORS) {
        ir_stop_triggered = true;
    }
    delay(500);
    //theni (ir1)
    //raba3 ir4
    //thelth (ir2)
    //lwl ir3
    // If already latched, do nothing — resetIRStop() clears it
}


// ─────────────────────────────────────────────────────────────
//  resetIRStop()
//  Call after the robot has acted on ir_stop_triggered
//  to allow the next detection event to latch again.
// ─────────────────────────────────────────────────────────────
void resetIRStop() {
    ir_stop_triggered = false;
}