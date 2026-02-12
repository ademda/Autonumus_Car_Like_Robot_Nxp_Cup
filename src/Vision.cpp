#include "vision.h"

// ---------- CONSTRUCTOR ---------- //
Vision::Vision() {
    calculated_angle = 90.0;
    FRAME_WIDTH = 0;
    FRAME_HEIGHT = 0;
    CENTER_X = 0;
}

// ---------- INITIALIZATION ---------- //
bool Vision::begin() {
    // Initialize Pixy2 with your custom SPI pins
    pixy.init();  // CS=10, MOSI=11, MISO=12, SCLK=13
    pixy.changeProg("line");
    //pixy.setLamp(1, 1);
    
    FRAME_WIDTH = pixy.frameWidth;
    FRAME_HEIGHT = pixy.frameHeight;
    CENTER_X = FRAME_WIDTH / 2.0;
    
    /*Serial.print("Frame: ");
    Serial.print(FRAME_WIDTH);
    Serial.print("x");
    Serial.print(FRAME_HEIGHT);
    Serial.print(" | Center: ");
    Serial.println(CENTER_X);
    Serial.println("------------------------------------------------------------");
    */
    pixy.setLamp(1, 1);
    return true;
}



float Vision::smooth_angle(float angle) {
    angle_history.push_back(angle);
    if (angle_history.size() > SMOOTHING_WINDOW) {
        angle_history.pop_front();
    }
    
    float sum = 0;
    for (size_t i = 0; i < angle_history.size(); i++) {
        sum += angle_history[i];
    }
    return sum / angle_history.size();
}

void Vision::sort_vectors_by_length(std::vector<VectorData>& vectors_list) {
    for (size_t i = 0; i < vectors_list.size(); i++) {
        for (size_t j = i + 1; j < vectors_list.size(); j++) {
            if (vectors_list[j].length > vectors_list[i].length) {
                VectorData temp = vectors_list[i];
                vectors_list[i] = vectors_list[j];
                vectors_list[j] = temp;
            }
        }
    }
}

float Vision::calculate_angle_degrees(float dx, float dy) {
    float angle = atan2(dy, dx) * (180.0 / M_PI);
    float normalized = fmod(angle + 180.0, 180.0);
    return normalized;
}

float Vision::vector_distance(const VectorData& v1, const VectorData& v2) {
    float x_mid1 = (v1.x0 + v1.x1) / 2;
    float y_mid1 = (v1.y0 + v1.y1) / 2;
    float x_mid2 = (v2.x0 + v2.x1) / 2;
    float y_mid2 = (v2.y0 + v2.y1) / 2;
    return sqrt((x_mid2 - x_mid1) * (x_mid2 - x_mid1) + (y_mid2 - y_mid1) * (y_mid2 - y_mid1));
}

// ---------- LINE FILTERING ---------- //
void Vision::filter_lines() {
    left_vectors.clear();
    right_vectors.clear();
    
    pixy.line.getAllFeatures();
    int v_count = pixy.line.numVectors;
    
    if (DEBUG_ANGLE) {
        //Serial.print("[DEBUG] Detected ");
        //Serial.print(v_count);
        //Serial.println(" line vectors");
    }
    
    for (int i = 0; i < v_count; i++) {
        float dx = pixy.line.vectors[i].m_x1 - pixy.line.vectors[i].m_x0;
        float dy = pixy.line.vectors[i].m_y1 - pixy.line.vectors[i].m_y0;
        float angle_deg = calculate_angle_degrees(dx, dy);
        float y_start = max(pixy.line.vectors[i].m_y0, pixy.line.vectors[i].m_y1);
        
        if (angle_deg > ANGLE_THRESHOLD && angle_deg < (180 - ANGLE_THRESHOLD) && y_start > FRAME_HEIGHT * 0.1) {
            float x0, y0, x1, y1;
            
            if (pixy.line.vectors[i].m_y0 > pixy.line.vectors[i].m_y1) {
                x0 = pixy.line.vectors[i].m_x0;
                y0 = pixy.line.vectors[i].m_y0;
                x1 = pixy.line.vectors[i].m_x1;
                y1 = pixy.line.vectors[i].m_y1;
            } else {
                x0 = pixy.line.vectors[i].m_x1;
                y0 = pixy.line.vectors[i].m_y1;
                x1 = pixy.line.vectors[i].m_x0;
                y1 = pixy.line.vectors[i].m_y0;
            }
            
            float length = sqrt((y1 - y0) * (y1 - y0) + (x1 - x0) * (x1 - x0));
            
            VectorData vector_data;
            vector_data.x0 = x0;
            vector_data.y0 = y0;
            vector_data.x1 = x1;
            vector_data.y1 = y1;
            vector_data.length = length;
            
            if (x0 < FRAME_WIDTH / 2) {
                left_vectors.push_back(vector_data);
            } else {
                right_vectors.push_back(vector_data);
            }
        }
    }
    
    sort_vectors_by_length(left_vectors);
    sort_vectors_by_length(right_vectors);
}

// ---------- STEERING CALCULATION ---------- //
float Vision::calculate_steering_angle(String& mode, float& distance) {
    filter_lines();
    int final_angle;
    float robot_distance = 0;
    
    if (left_vectors.size() >= 1 && right_vectors.size() >= 1) {
        VectorData left = left_vectors[0];
        VectorData right = right_vectors[0];
        
        float x_dist = vector_distance(left, right);
        
        if (DEBUG_MERGE) {
            Serial.print("[DEBUG] BOTH VECTORS: Left length=");
            Serial.print(left.length, 1);
            Serial.print(", Right length=");
            Serial.print(right.length, 1);
            Serial.print(", X Dist=");
            Serial.println(x_dist, 1);
        }
        
        if (x_dist < MERGE_THRESHOLD) {
            // Merge vectors as one
            float mid_x0 = (left.x0 + left.x1 + right.x0 + right.x1) / 4;
            float mid_x1 = mid_x0;
            float mid_y0 = (left.y0 + left.y1 + right.y0 + right.y1) / 4;
            float mid_y1 = mid_y0 + 1;  // small vertical vector
            robot_distance = mid_x0 - CENTER_X;
            
            float dx = mid_x1 - mid_x0;
            float dy = mid_y1 - mid_y0;
            
            float angle_deg = calculate_angle_degrees(dx, dy);
            robot_distance = constrain(robot_distance, -15, 15);
            calculated_angle = angle_deg + (K_lateral * robot_distance);

            
            if (DEBUG_ANGLE) {
                Serial.println("[DEBUG] BOTH_SINGLE MERGED VECTORS");
                Serial.print("        mid_x0=");
                Serial.print(mid_x0, 1);
                Serial.print(", mid_y0=");
                Serial.print(mid_y0, 1);
                Serial.print(", dx=");
                Serial.print(dx, 1);
                Serial.print(", dy=");
                Serial.println(dy, 1);
                Serial.print("        angle_deg=");
                Serial.print(angle_deg, 2);
                Serial.print(", lateral_error=");
                Serial.print(robot_distance, 2);
                Serial.print(", final_angle=");
                Serial.println(calculated_angle, 2);
            }
            
            mode = "BOTH_SINGLE";
            distance = robot_distance;
            return calculated_angle;
        } else {
            // Normal BOTH
            float mid_x0 = (left.x0 + right.x0) / 2;
            float mid_x1 = (left.x1 + right.x1) / 2;
            float mid_y0 = (left.y0 + right.y0) / 2;
            float mid_y1 = (left.y1 + right.y1) / 2;
            
            robot_distance = mid_x0 - CENTER_X;
            float dx = mid_x1 - mid_x0;
            float dy = mid_y1 - mid_y0;
            
            float angle_deg = calculate_angle_degrees(dx, dy);
            robot_distance = constrain(robot_distance, -15, 15);
            calculated_angle = angle_deg + robot_distance;
            
            if (DEBUG_ANGLE) {
                Serial.println("[DEBUG] BOTH VECTORS");
                Serial.print("        left=(");
                Serial.print(left.x0);
                Serial.print(",");
                Serial.print(left.y0);
                Serial.print(")-(");
                Serial.print(left.x1);
                Serial.print(",");
                Serial.print(left.y1);
                Serial.println(")");
                Serial.print("        right=(");
                Serial.print(right.x0);
                Serial.print(",");
                Serial.print(right.y0);
                Serial.print(")-(");
                Serial.print(right.x1);
                Serial.print(",");
                Serial.print(right.y1);
                Serial.println(")");
                Serial.print("        mid_x0=");
                Serial.print(mid_x0, 1);
                Serial.print(", mid_y0=");
                Serial.print(mid_y0, 1);
                Serial.print(", dx=");
                Serial.print(dx, 1);
                Serial.print(", dy=");
                Serial.println(dy, 1);
                Serial.print("        angle_deg=");
                Serial.print(angle_deg, 2);
                Serial.print(", lateral_error=");
                Serial.print(robot_distance, 2);
                Serial.print(", final_angle=");
                Serial.println(calculated_angle, 2);
            }
            
            mode = "BOTH";
            distance = robot_distance;
            return calculated_angle;
        }
    } else if (left_vectors.size() >= 1 && right_vectors.size() == 0) {
        VectorData left = left_vectors[0];
        float dx = left.x1 - left.x0;
        float dy = left.y1 - left.y0;
        float angle_deg = calculate_angle_degrees(dx, dy);
        calculated_angle = angle_deg;
        
        if (DEBUG_ANGLE) {
            Serial.println("[DEBUG] LEFT VECTOR");
            Serial.print("        left=(");
            Serial.print(left.x0);
            Serial.print(",");
            Serial.print(left.y0);
            Serial.print(")-(");
            Serial.print(left.x1);
            Serial.print(",");
            Serial.print(left.y1);
            Serial.println(")");
            Serial.print("        dx=");
            Serial.print(dx, 1);
            Serial.print(", dy=");
            Serial.print(dy, 1);
            Serial.print(", angle_deg=");
            Serial.print(angle_deg, 2);
            Serial.print(", final_angle=");
            Serial.println(calculated_angle, 2);
        }
        
        mode = "LEFT";
        distance = 0;
        return calculated_angle;
    } else if (left_vectors.size() == 0 && right_vectors.size() >= 1) {
        VectorData right = right_vectors[0];
        float dx = right.x1 - right.x0;
        float dy = right.y1 - right.y0;
        float angle_deg = calculate_angle_degrees(dx, dy);
        calculated_angle = angle_deg;
        
        if (DEBUG_ANGLE) {
            Serial.println("[DEBUG] RIGHT VECTOR");
            Serial.print("        right=(");
            Serial.print(right.x0);
            Serial.print(",");
            Serial.print(right.y0);
            Serial.print(")-(");
            Serial.print(right.x1);
            Serial.print(",");
            Serial.print(right.y1);
            Serial.println(")");
            Serial.print("        dx=");
            Serial.print(dx, 1);
            Serial.print(", dy=");
            Serial.print(dy, 1);
            Serial.print(", angle_deg=");
            Serial.print(angle_deg, 2);
            Serial.print(", final_angle=");
            Serial.println(calculated_angle, 2);
        }
        
        mode = "RIGHT";
        distance = 0;
        return calculated_angle;
    } else {
        if (DEBUG_ANGLE) {
            Serial.print("[DEBUG] LOST - No vectors detected, keeping previous angle=");
            Serial.println(calculated_angle, 2);
        }
        
        mode = "LOST";
        distance = 0;
        return calculated_angle;
    }
}

// ---------- SERVO ANGLE CALCULATION ---------- //
int Vision::get_servo_angle(float angle) {
    float final_angle = constrain(angle, 30, 150);
    int servo_angle = constrain(final_angle, 55, 117);;
    servo_angle = (int)smooth_angle((float)servo_angle);
    return servo_angle;
}

// ---------- CLEANUP ---------- //
void Vision::cleanup() {
    pixy.setLamp(0, 0);
    Serial.println("\nShutdown...");
}