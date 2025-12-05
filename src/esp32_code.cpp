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
  // PID parameters
  float vel_kp, vel_ki, vel_kd;
  float steer_kp, steer_ki, steer_kd;
};

CarData received_data = {0.0, 0.0, 0.0, false, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0};

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 3, 1);
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
  
  // Check for incoming status from Teensy
  receiveTeensyStatus();
  
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
        
        // Print current data
        printReceivedData();
      }
    }
    
    client.stop();
    Serial.println("Client disconnected");
  }
}

void parseReceivedData(String data) {
  // Expected format: "right_vel,left_vel,distance,emergency,vel_kp,vel_ki,vel_kd,steer_kp,steer_ki,steer_kd"
  // Example: "100.5,200.0,500.0,0,1.500,0.100,0.050,2.000,0.200,0.080"
  
  int commaPositions[9];
  int commaCount = 0;
  
  // Find all comma positions
  for (int i = 0; i < data.length() && commaCount < 9; i++) {
    if (data[i] == ',') {
      commaPositions[commaCount] = i;
      commaCount++;
    }
  }
  
  if (commaCount >= 9) {
    // Parse all 10 values
    received_data.right_velocity = data.substring(0, commaPositions[0]).toFloat();
    received_data.left_velocity = data.substring(commaPositions[0] + 1, commaPositions[1]).toFloat();
    received_data.distance = data.substring(commaPositions[1] + 1, commaPositions[2]).toFloat();
    received_data.emergency_stop = data.substring(commaPositions[2] + 1, commaPositions[3]).toInt() == 1;
    
    // Parse PID values
    received_data.vel_kp = data.substring(commaPositions[3] + 1, commaPositions[4]).toFloat();
    received_data.vel_ki = data.substring(commaPositions[4] + 1, commaPositions[5]).toFloat();
    received_data.vel_kd = data.substring(commaPositions[5] + 1, commaPositions[6]).toFloat();
    received_data.steer_kp = data.substring(commaPositions[6] + 1, commaPositions[7]).toFloat();
    received_data.steer_ki = data.substring(commaPositions[7] + 1, commaPositions[8]).toFloat();
    received_data.steer_kd = data.substring(commaPositions[8] + 1).toFloat();
    // Send PID data to Teensy via UART
  } else if (commaCount >= 3) {
    // Fallback for old format (only velocity/distance data)
    received_data.right_velocity = data.substring(0, commaPositions[0]).toFloat();
    received_data.left_velocity = data.substring(commaPositions[0] + 1, commaPositions[1]).toFloat();
    received_data.distance = data.substring(commaPositions[1] + 1, commaPositions[2]).toFloat();
    received_data.emergency_stop = data.substring(commaPositions[2] + 1).toInt() == 1;
  }
  sendPIDToTeensy();
}

void printReceivedData() {
  Serial.println("=== Current Data ===");
  Serial.println("Right Velocity: " + String(received_data.right_velocity) + " mm/s");
  Serial.println("Left Velocity: " + String(received_data.left_velocity) + " mm/s");
  Serial.println("Distance: " + String(received_data.distance) + " mm");
  Serial.println("Emergency Stop: " + String(received_data.emergency_stop ? "YES" : "NO"));
  Serial.println("Velocity PID: Kp=" + String(received_data.vel_kp, 3) + 
                 ", Ki=" + String(received_data.vel_ki, 3) + 
                 ", Kd=" + String(received_data.vel_kd, 3));
  Serial.println("Steering PID: Kp=" + String(received_data.steer_kp, 3) + 
                 ", Ki=" + String(received_data.steer_ki, 3) + 
                 ", Kd=" + String(received_data.steer_kd, 3));
  Serial.println("==================");
}

// Function to get received data (call this from your main control code)
CarData getReceivedData() {
  return received_data;
}

void sendPIDToTeensy() {
  // Send PID values to Teensy via UART
  // Format: "vel_kp,vel_ki,vel_kd,steer_kp,steer_ki,steer_kd,right_vel,left_vel,distance,emergency"
  String uartData = String(received_data.vel_kp, 3) + "," + 
                   String(received_data.vel_ki, 3) + "," + 
                   String(received_data.vel_kd, 3) + "," + 
                   String(received_data.steer_kp, 3) + "," + 
                   String(received_data.steer_ki, 3) + "," + 
                   String(received_data.steer_kd, 3) + "," +
                   String(received_data.right_velocity, 1) + "," +
                   String(received_data.left_velocity, 1) + "," +
                   String(received_data.distance, 1) + "," +
                   String(received_data.emergency_stop ? 1 : 0) + "\n";
  
  Serial1.print(uartData);
  Serial.println("Sent to Teensy: " + uartData.substring(0, uartData.length()-1)); // Remove newline for display
}

void receiveTeensyStatus() {
  // Check for incoming data from Teensy
  if (Serial1.available()) {
    String statusData = Serial1.readStringUntil('\n');
    statusData.trim();
    Serial.println("Teensy Status: " + statusData);
    //adem : here i need to send data over wifi to plot it

    // Parse Teensy status if needed
    // Format: "robot_vel,orientation,left_vel,right_vel,servo_angle"
    // You can add parsing here to extract actual robot status
  }
}