#include <WiFi.h>

// WiFi credentials
const char* ssid = "NxP_Cup_Car";
const char* password = "12345678";

// TCP server on port 8080
WiFiServer server(8080);

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

void setup() {
  Serial.begin(115200);
  delay(1000);
  
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
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("New client connected");
    
    while (client.connected()) {
      if (client.available()) {
        String receivedData = client.readStringUntil('\n');
        receivedData.trim();
        
        Serial.println("Received: " + receivedData);
        
        // Parse the received data
        parseReceivedData(receivedData);
        
        // Send acknowledgment back to PC
        client.println("Data received: " + receivedData);
        
        // Print current data and send to Teensy
        printReceivedData();
        sendToTeensy();
      }
    }
    
    client.stop();
    Serial.println("Client disconnected");
  }
}

void parseReceivedData(String data) {
  // Expected format: "right_vel,left_vel,distance,emergency,right_kp,right_ki,right_kd,left_kp,left_ki,left_kd,steer_kp,steer_ki,steer_kd,distance_mode"
  // Example: "100.5,200.0,500.0,0,1.5,0.1,0.05,1.5,0.1,0.05,2.0,0.2,0.08,1"
  
  int commaPositions[13];
  int commaCount = 0;
  
  // Find all comma positions
  for (int i = 0; i < data.length() && commaCount < 13; i++) {
    if (data[i] == ',') {
      commaPositions[commaCount] = i;
      commaCount++;
    }
  }
  
  if (commaCount >= 13) {
    // Parse new 14-value format with separate left/right motor PID
    received_data.right_velocity = data.substring(0, commaPositions[0]).toFloat();
    received_data.left_velocity = data.substring(commaPositions[0] + 1, commaPositions[1]).toFloat();
    received_data.distance = data.substring(commaPositions[1] + 1, commaPositions[2]).toFloat();
    received_data.emergency_stop = data.substring(commaPositions[2] + 1, commaPositions[3]).toInt() == 1;
    
    // Parse RIGHT motor PID values
    received_data.right_vel_kp = data.substring(commaPositions[3] + 1, commaPositions[4]).toFloat();
    received_data.right_vel_ki = data.substring(commaPositions[4] + 1, commaPositions[5]).toFloat();
    received_data.right_vel_kd = data.substring(commaPositions[5] + 1, commaPositions[6]).toFloat();
    
    // Parse LEFT motor PID values
    received_data.left_vel_kp = data.substring(commaPositions[6] + 1, commaPositions[7]).toFloat();
    received_data.left_vel_ki = data.substring(commaPositions[7] + 1, commaPositions[8]).toFloat();
    received_data.left_vel_kd = data.substring(commaPositions[8] + 1, commaPositions[9]).toFloat();
    
    // Parse Steering PID values
    received_data.steer_kp = data.substring(commaPositions[9] + 1, commaPositions[10]).toFloat();
    received_data.steer_ki = data.substring(commaPositions[10] + 1, commaPositions[11]).toFloat();
    received_data.steer_kd = data.substring(commaPositions[11] + 1, commaPositions[12]).toFloat();
    
    // Parse distance mode
    received_data.distance_mode = data.substring(commaPositions[12] + 1).toInt() == 1;
  } else if (commaCount >= 3) {
    // Fallback for old format (only velocity/distance data)
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
  // Send to Teensy in format: "right_kp,right_ki,right_kd,left_kp,left_ki,left_kd,steer_kp,steer_ki,steer_kd,right_velocity,left_velocity,distance,emergency,distance_mode"
  // This format matches what Teensy parseTuningValues expects
  Serial.print("Sending to Teensy: ");
  
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
  
  Serial.println(teensyData);
}

// Function to get received data (call this from your main control code)
CarData getReceivedData() {
  return received_data;
}