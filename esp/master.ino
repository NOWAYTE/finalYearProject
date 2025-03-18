#include <SoftwareSerial.h>

// Define SoftwareSerial pins for communication with the Slave Arduino
// Here, pin 10 is used for RX and pin 11 for TX
SoftwareSerial slaveSerial(10, 11);  

void setup() {
  // Initialize hardware Serial for communication with the ESP32
  // This may also be used for debugging via the Serial Monitor
  Serial.begin(9600);
  
  // Initialize SoftwareSerial for communication with the Slave Arduino
  slaveSerial.begin(9600);
  
  // Debug message indicating that the Master Arduino has started
  Serial.println("Master Arduino started, awaiting commands from ESP32...");
}

void loop() {
  // Check for incoming command data from the ESP32 on hardware Serial
  if (Serial.available()) {
    // Read the incoming command until a newline character is encountered
    String command = Serial.readStringUntil('\n');
    command.trim();  // Remove any leading/trailing whitespace or newline characters
    
    // If the command string is not empty, process it
    if (command.length() > 0) {
      // Debug: Print the received command
      Serial.print("Received command from ESP32: ");
      Serial.println(command);
      
      // Forward the command to the Slave Arduino via SoftwareSerial
      slaveSerial.println(command);
    }
  }
  
  // (Optional) If you need to read any responses from the Slave Arduino, you can do so here
  if (slaveSerial.available()) {
    String slaveResponse = slaveSerial.readStringUntil('\n');
    Serial.print("Response from Slave: ");
    Serial.println(slaveResponse);
  }
}

