#ifndef VISION_H
#define VISION_H

#include <Pixy2.h>
#include <vector>
#include <deque>
#include <cmath>

// ---------- CONFIG ---------- //
#define MAX_ANGLE 45
#define DEADZONE 1.5
#define SMOOTHING_WINDOW 15
#define ANGLE_THRESHOLD 25 //15
#define LINE_VECTOR_SIZE 20
#define DEFAULT_SERVO_ANGLE 87
#define MERGE_THRESHOLD 10  // pixels, merge vectors if distance is small
#define DEBUG_ANGLE 0 // 1=on, 0=off: detailed angle debugging
#define DEBUG_MERGE 0  // 1=on, 0=off: detailed merge debugging
#define K_lateral 0.8  // initial value

// ---------- VECTOR STRUCTURE ---------- //
struct VectorData {
    float x0;
    float y0;
    float x1;
    float y1;
    float length;
};

// ---------- VISION CLASS ---------- //
class Vision {
public :
    // Pixy2 object
    Pixy2 pixy;
    
    // Frame dimensions
    int FRAME_WIDTH;
    int FRAME_HEIGHT;
    float CENTER_X;
    
    // Vectors storage
    std::vector<VectorData> left_vectors;
    std::vector<VectorData> right_vectors;
    
    // Smoothing
    std::deque<float> angle_history;
    
    // Current state
    float calculated_angle;
    
    // Utility functions
    float smooth_angle(float angle);
    void sort_vectors_by_length(std::vector<VectorData>& vectors_list);
    float calculate_angle_degrees(float dx, float dy);
    float vector_distance(const VectorData& v1, const VectorData& v2);
    
    // Line filtering
    void filter_lines();
    
public:
    // Constructor
    Vision();
    
    // Initialization
    bool begin();
    
    // Main calculation
    float calculate_steering_angle(String& mode, float& distance);
    
    // Servo angle calculation
    int get_servo_angle(float angle);
    
    // Cleanup
    void cleanup();
};

#endif