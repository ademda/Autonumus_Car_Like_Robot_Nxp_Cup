#include "tof.h"

// ─────────────────────────────────────────────────────────────
//  PUBLIC VARIABLE DEFINITIONS
// ─────────────────────────────────────────────────────────────

float left_tof_distance = 0;
float center_tof_distance = 0;
float right_tof_distance = 0;
bool cube_detected = false;
VL53L0X_RangingMeasurementData_t tof_measure;
ToFFilter tofFilter;
Adafruit_VL53L0X tof1 = Adafruit_VL53L0X();
uint32_t last_tof_test = 0;

// ─────────────────────────────────────────────────────────────
//  PUBLIC FUNCTIONS
// ─────────────────────────────────────────────────────────────

void ToF_Init(TwoWire* wireInterface, uint8_t sda_pin, uint8_t scl_pin) {
    wireInterface->begin();
    wireInterface->setSDA(sda_pin);
    wireInterface->setSCL(scl_pin);
    wireInterface->setClock(400000); // Fast I2C

    if (!tof1.begin(0x29, wireInterface)) {
        Serial.println("Failed to boot ToF 1");
    }
    tof1.setAddress(TOF_ADDR_1);
    tof1.startRangeContinuous(50);

    /* Filter Initialisation */
    tofFilter.setOffset(15);
    tofFilter.setRangeLimits(20, 20000);
    tofFilter.setPublishInterval(1000 / TOF_INTERVAL_MS); // 2 Hz max
}

void readToFsNonBlocking() {
    if (millis() - last_tof_test >= TOF_INTERVAL_MS) {
        uint16_t distance;

        // ----- ToF1 -----
        if (tof1.isRangeComplete()) {
            distance = tof1.readRange();
            if (!tof1.timeoutOccurred()) {
                center_tof_distance = tofFilter.filter(distance) * 1000; // convert to mm
            }
        }

        if (center_tof_distance < STOP_DISTANCE) {
            cube_detected = true;
        } else if (center_tof_distance > STOP_DISTANCE) {
            cube_detected = false;
        }
        last_tof_test = millis();

#ifdef DEBUG
        Serial.print("left_tof_distance: ");
        Serial.print(left_tof_distance);
        Serial.print(" mm | ");
        Serial.print("center_tof_distance: ");
        Serial.print(center_tof_distance);
        Serial.print(" mm | ");
        Serial.print("right_tof_distance: ");
        Serial.print(right_tof_distance);
        Serial.println(" mm | ");
#endif
    }
}

void resetCubeDetected() {
    cube_detected = false;
}
