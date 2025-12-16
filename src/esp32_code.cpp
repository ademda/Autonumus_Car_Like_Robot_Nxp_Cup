#include <WiFi.h>
WiFiClient client;
unsigned long lastStatusSend = 0;
const unsigned long STATUS_PERIOD_MS = 100;
// WiFi credentials
const char* ssid = "NxP_Cup_Car";
const char* password = "12345678";

// TCP server on port 8080
WiFiServer server(8080);

// Pins for Teensy UART (Serial2)
#define TX2_PIN 17  // ESP32 TX2 → Teensy RX
#define RX2_PIN 16  // ESP32 RX2 ← Teensy TX

// Data structure to store received data
struct CarData {
  float right_velocity;
  float left_velocity;
  float distance;
  bool emergency_stop;
  // Separate PID parameters for each motor
  float right_vel_kp, right_vel_ki, right_vel_kd;
  float left_vel_kp, left_vel_ki, left_vel_kd;
  float steer_kp, steer_ki, steer_kd;
  bool distance_mode;
};

CarData received_data = {0.0, 0.0, 0.0, false, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, false};

// Data structure to store Teensy status
struct TeensyStatus {
  float robot_distance_mm;
  float left_wheel_curr_vel_mm_s;
  float right_wheel_curr_vel_mm_s;
  float left_wheel_distance_mm;
  float right_wheel_distance_mm;
};

TeensyStatus teensy_status = {0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
  // Serial monitor
  Serial.begin(115200);
  delay(1000);

  // UART to Teensy
  Serial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);

  // Create WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Start TCP server
  server.begin();
  Serial.println("TCP server started on port 8080");
}

void loop() {

  // Accept new client (non-blocking)
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connected");
    }
  }

  // 1. Handle incoming commands from PC
  if (client && client.connected() && client.available()) {

    String receivedData = client.readStringUntil('\n');
    receivedData.trim();

    Serial.println("Received: " + receivedData);

    parseReceivedData(receivedData);
    printReceivedData();
    sendToTeensy();
  }

  // 2. Always receive Teensy status (REAL or simulated)
  //receiveTeensyStatusSimulation();   // or 
  receiveTeensyStatus();
  // 3. Periodically stream status to PC
  if (client && client.connected()) {
    if (millis() - lastStatusSend >= STATUS_PERIOD_MS) {
      lastStatusSend = millis();

      String statusResponse =
        String(teensy_status.robot_distance_mm, 2) + "," +
        String(teensy_status.left_wheel_curr_vel_mm_s, 2) + "," +
        String(teensy_status.right_wheel_curr_vel_mm_s, 2) + "," +
        String(teensy_status.left_wheel_distance_mm, 2) + "," +
        String(teensy_status.right_wheel_distance_mm, 2);

      client.println(statusResponse);
    }
  }
}

void parseReceivedData(String data) {
  int commaPositions[13];
  int commaCount = 0;
  
  for (int i = 0; i < data.length() && commaCount < 13; i++) {
    if (data[i] == ',') {
      commaPositions[commaCount++] = i;
    }
  }
  
  if (commaCount >= 13) {
    received_data.right_velocity = data.substring(0, commaPositions[0]).toFloat();
    received_data.left_velocity = data.substring(commaPositions[0] + 1, commaPositions[1]).toFloat();
    received_data.distance = data.substring(commaPositions[1] + 1, commaPositions[2]).toFloat();
    received_data.emergency_stop = data.substring(commaPositions[2] + 1, commaPositions[3]).toInt() == 1;
    
    received_data.right_vel_kp = data.substring(commaPositions[3] + 1, commaPositions[4]).toFloat();
    received_data.right_vel_ki = data.substring(commaPositions[4] + 1, commaPositions[5]).toFloat();
    received_data.right_vel_kd = data.substring(commaPositions[5] + 1, commaPositions[6]).toFloat();
    
    received_data.left_vel_kp = data.substring(commaPositions[6] + 1, commaPositions[7]).toFloat();
    received_data.left_vel_ki = data.substring(commaPositions[7] + 1, commaPositions[8]).toFloat();
    received_data.left_vel_kd = data.substring(commaPositions[8] + 1, commaPositions[9]).toFloat();
    
    received_data.steer_kp = data.substring(commaPositions[9] + 1, commaPositions[10]).toFloat();
    received_data.steer_ki = data.substring(commaPositions[10] + 1, commaPositions[11]).toFloat();
    received_data.steer_kd = data.substring(commaPositions[11] + 1, commaPositions[12]).toFloat();
    
    received_data.distance_mode = data.substring(commaPositions[12] + 1).toInt() == 1;
  } else if (commaCount >= 3) {
    received_data.right_velocity = data.substring(0, commaPositions[0]).toFloat();
    received_data.left_velocity = data.substring(commaPositions[0] + 1, commaPositions[1]).toFloat();
    received_data.distance = data.substring(commaPositions[1] + 1, commaPositions[2]).toFloat();
    received_data.emergency_stop = data.substring(commaPositions[2] + 1).toInt() == 1;
  }
}

void printReceivedData() {
  Serial.println("=== Current Data ===");
  Serial.println("Right Velocity: " + String(received_data.right_velocity) + " mm/s");
  Serial.println("Left Velocity: " + String(received_data.left_velocity) + " mm/s");
  Serial.println("Distance: " + String(received_data.distance) + " mm");
  Serial.println("Emergency Stop: " + String(received_data.emergency_stop ? "YES" : "NO"));
  Serial.println("Right Motor PID: Kp=" + String(received_data.right_vel_kp, 3) + 
                 ", Ki=" + String(received_data.right_vel_ki, 3) + 
                 ", Kd=" + String(received_data.right_vel_kd, 3));
  Serial.println("Left Motor PID: Kp=" + String(received_data.left_vel_kp, 3) + 
                 ", Ki=" + String(received_data.left_vel_ki, 3) + 
                 ", Kd=" + String(received_data.left_vel_kd, 3));
  Serial.println("Steering PID: Kp=" + String(received_data.steer_kp, 3) + 
                 ", Ki=" + String(received_data.steer_ki, 3) + 
                 ", Kd=" + String(received_data.steer_kd, 3));
  Serial.println("Distance Mode: " + String(received_data.distance_mode ? "ENABLED" : "DISABLED"));
  Serial.println("==================");
}

void sendToTeensy() {
  String teensyData = String(received_data.right_vel_kp, 3) + "," +
                      String(received_data.right_vel_ki, 3) + "," +
                      String(received_data.right_vel_kd, 3) + "," +
                      String(received_data.left_vel_kp, 3) + "," +
                      String(received_data.left_vel_ki, 3) + "," +
                      String(received_data.left_vel_kd, 3) + "," +
                      String(received_data.steer_kp, 3) + "," +
                      String(received_data.steer_ki, 3) + "," +
                      String(received_data.steer_kd, 3) + "," +
                      String(received_data.right_velocity, 1) + "," +
                      String(received_data.left_velocity, 1) + "," +
                      String(received_data.distance, 1) + "," +
                      String(received_data.emergency_stop ? 1 : 0) + "," +
                      String(received_data.distance_mode ? 1 : 0) + "\n";
  
  Serial2.print(teensyData);       // send to Teensy via UART2
  Serial.print("Sent to Teensy: "); // print for debugging
  Serial.println(teensyData);
}

CarData getReceivedData() {
  return received_data;
}

// Receive status from Teensy via UART
void receiveTeensyStatus() {
  // Non-blocking receive - only read if data is available
  while (Serial2.available()) {
    String statusData = Serial2.readStringUntil('\n');
    statusData.trim();
    
    if (statusData.length() > 0) {
      Serial.println("Teensy Status: " + statusData);
      parseTeensyStatus(statusData);
    }
  }
}



// Parse Teensy status data
void parseTeensyStatus(String data) {
  // Expected format: "distance,orientation,left_vel,right_vel,servo_angle"
  int commaPositions[4];
  int commaCount = 0;
  
  for (int i = 0; i < data.length() && commaCount < 4; i++) {
    if (data[i] == ',') {
      commaPositions[commaCount++] = i;
    }
  }
  
  if (commaCount >= 4) {
    teensy_status.robot_distance_mm = data.substring(0, commaPositions[0]).toFloat();
    teensy_status.left_wheel_curr_vel_mm_s = data.substring(commaPositions[0] + 1, commaPositions[1]).toFloat();
    teensy_status.right_wheel_curr_vel_mm_s = data.substring(commaPositions[1] + 1, commaPositions[2]).toFloat();
    teensy_status.left_wheel_distance_mm = data.substring(commaPositions[2] + 1, commaPositions[3]).toFloat();
    teensy_status.right_wheel_distance_mm = data.substring(commaPositions[3] + 1).toFloat();
    
    // Print parsed status for debugging
    /*Serial.println("--- Teensy Status Parsed ---");
    Serial.println("Distance: " + String(teensy_status.robot_distance_mm) + " mm");
    Serial.println("Orientation: " + String(teensy_status.left_wheel_curr_vel_mm_s) + "°");
    Serial.println("Left Vel: " + String(teensy_status.right_wheel_curr_vel_mm_s) + " mm/s");
    Serial.println("Right Vel: " + String(teensy_status.left_wheel_distance_mm) + " mm/s");
    Serial.println("Servo: " + String(teensy_status.right_wheel_distance_mm) + "°");*/
  }
}

// Getter function to access Teensy status
TeensyStatus getTeensyStatus() {
  return teensy_status;
}
