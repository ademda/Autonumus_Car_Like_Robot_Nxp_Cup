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
};

CarData received_data = {0.0, 0.0, 0.0, false};

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
        
        // Print current data
        printReceivedData();
      }
    }
    
    client.stop();
    Serial.println("Client disconnected");
  }
}

void parseReceivedData(String data) {
  // Expected format: "right_vel,left_vel,distance,emergency"
  // Example: "100.5,200.0,500.0,0"
  
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);
  int thirdComma = data.indexOf(',', secondComma + 1);
  
  if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
    received_data.right_velocity = data.substring(0, firstComma).toFloat();
    received_data.left_velocity = data.substring(firstComma + 1, secondComma).toFloat();
    received_data.distance = data.substring(secondComma + 1, thirdComma).toFloat();
    received_data.emergency_stop = data.substring(thirdComma + 1).toInt() == 1;
  }
}

void printReceivedData() {
  Serial.println("=== Current Data ===");
  Serial.println("Right Velocity: " + String(received_data.right_velocity) + " mm/s");
  Serial.println("Left Velocity: " + String(received_data.left_velocity) + " mm/s");
  Serial.println("Distance: " + String(received_data.distance) + " mm");
  Serial.println("Emergency Stop: " + String(received_data.emergency_stop ? "YES" : "NO"));
  Serial.println("==================");
}

// Function to get received data (call this from your main control code)
CarData getReceivedData() {
  return received_data;
}