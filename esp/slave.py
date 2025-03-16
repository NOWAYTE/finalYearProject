#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define SoftwareSerial pins for the Slave
// Here, pin 2 is used as RX (to receive data from the Master)
// Pin 3 is defined as TX (only needed if sending data back)
SoftwareSerial slaveSerial(2, 3);  // RX, TX

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x47);

// Function to move forward
void advance() {
  pwm.setPWM(0, 0, 4095);
  pwm.setPWM(1, 0, 1000);
  pwm.setPWM(2, 0, 4095);
  pwm.setPWM(3, 0, 1000);
  pwm.setPWM(4, 0, 4095);
  pwm.setPWM(5, 0, 1000);
  pwm.setPWM(6, 0, 4095);
  pwm.setPWM(7, 0, 1000);
}

// Function to turn right
void turnR() {
  pwm.setPWM(0, 0, 4095);
  pwm.setPWM(1, 0, 2000);
  pwm.setPWM(2, 0, 4095);
  pwm.setPWM(3, 0, 2000);
  pwm.setPWM(4, 0, 0);
  pwm.setPWM(5, 0, 2000);
  pwm.setPWM(6, 0, 0);
  pwm.setPWM(7, 0, 2000);
}

// Function to turn left
void turnL() {
  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, 1000);
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 1000);
  pwm.setPWM(4, 0, 4095);
  pwm.setPWM(5, 0, 1000);
  pwm.setPWM(6, 0, 4095);
  pwm.setPWM(7, 0, 1000);
}

// Function to stop the motors
void stopp() {
  pwm.setPWM(1, 0, 0);
  pwm.setPWM(3, 0, 0);
  pwm.setPWM(5, 0, 0);
  pwm.setPWM(7, 0, 0);
}

// Function to move backward
void back() {
  pwm.setPWM(0, 0, 0);
  pwm.setPWM(1, 0, 2000);
  pwm.setPWM(2, 0, 0);
  pwm.setPWM(3, 0, 2000);
  pwm.setPWM(4, 0, 0);
  pwm.setPWM(5, 0, 2000);
  pwm.setPWM(6, 0, 0);
  pwm.setPWM(7, 0, 2000);
}

void setup() {
  // Initialize SoftwareSerial for receiving commands from the Master Arduino
  slaveSerial.begin(9600);
  
  // Initialize hardware Serial for debugging (optional)
  Serial.begin(9600);
  
  // Initialize the PWM driver
  pwm.begin();
  pwm.setPWMFreq(60);
  
  // Ensure the car starts stopped
  stopp();
  
  Serial.println("Slave Arduino with SoftwareSerial Ready");
}

void loop() {
  if (slaveSerial.available()) {
    String command = slaveSerial.readStringUntil('\n');
    command.trim();  // Remove any extra whitespace
    
    // Debug print the received command
    Serial.print("Received command: ");
    Serial.println(command);
    
    // Execute motor control based on the command
    if (command.equalsIgnoreCase("forward")) {
      advance();
    } else if (command.equalsIgnoreCase("back")) {
      back();
    } else if (command.equalsIgnoreCase("left")) {
      turnL();
    } else if (command.equalsIgnoreCase("right")) {
      turnR();
    } else if (command.equalsIgnoreCase("stop")) {
      stopp();
    } else {
      Serial.println("Unknown command");
    }
  }
  
  delay(10);  // Small delay for stability
}

