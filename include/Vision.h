#ifndef VISION_H
#define VISION_H

#include <Pixy2.h>
#include <vector>
#include <deque>
#include <cmath>
#include <Arduino.h> // Ensure String is recognized

// ---------- CONFIG ---------- //
#define MAX_ANGLE 45
#define DEADZONE 1.5
#define SMOOTHING_WINDOW 15
#define ANGLE_THRESHOLD 25 
#define LINE_VECTOR_SIZE 20
#define DEFAULT_SERVO_ANGLE 87
#define MERGE_THRESHOLD 10  
#define DEBUG_ANGLE 0 
#define DEBUG_MERGE 0  
#define K_lateral 0.8  

// --- DISTORTION CONFIG --- //
// Tuned value from Python visualizer (79x52 line-mode coordinate space).
// At top of frame (y=0):  correction_factor = 1 + 0.0048 * 52 = 1.250  (25% stretch)
// At bottom (y=52):       correction_factor = 1.000  (no change)
// Increase K_DISTORT if lines still lean inward at the top.
// Decrease K_DISTORT if lines bow outward like a 'V'.
#define K_DISTORT 0.0005f //0.0048f
// Only correct vectors longer than this (px, in 79x52 space).
// Typical real lane vectors are 15-25px.  10px keeps almost everything corrected
// while ignoring truly tiny noise fragments.
#define MIN_LENGTH_FOR_CORRECTION 10.0f

struct VectorData {
    float x0;
    float y0;
    float x1;
    float y1;
    float length;
};

class Vision {
private:
    // Perspective correction helper
    float apply_perspective_correction(float x, float y);

public:
    Pixy2 pixy;
    int FRAME_WIDTH;
    int FRAME_HEIGHT;
    float CENTER_X;
    
    std::vector<VectorData> left_vectors;
    std::vector<VectorData> right_vectors;
    std::deque<float> angle_history;
    float calculated_angle;
    
    // Utility functions
    float smooth_angle(float angle);
    void sort_vectors_by_length(std::vector<VectorData>& vectors_list);
    float calculate_angle_degrees(float dx, float dy);
    float vector_distance(const VectorData& v1, const VectorData& v2);
    
    // Line filtering
    void filter_lines();

    // Constructor & Lifecycle
    Vision();
    bool begin();
    
    // Main calculation
    float calculate_steering_angle(String& mode, float& distance);
    int get_servo_angle(float angle);
    void cleanup();
};

#endif