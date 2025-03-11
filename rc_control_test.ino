    // Pins for Motor Driver (Example: L298N)
const int motorA1 = 5;  // Motor A Direction 1
const int motorA2 = 6;  // Motor A Direction 2
const int motorB1 = 9;  // Motor B Direction 1
const int motorB2 = 10; // Motor B Direction 2

void setup() {
  Serial.begin(115200); // Match baud rate with Python script
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  stopMotors(); // Initialize with motors stopped
}

void loop() {
  if (Serial.available() > 0) {
    int command = Serial.read();
    executeCommand(command);
  }
}

void executeCommand(int cmd) {
  switch (cmd) {
    case 0: stopMotors();       break; // Stop
    case 1: moveForward();       break; // Forward
    case 2: moveReverse();       break; // Reverse
    case 3: turnRight();         break; // Right
    case 4: turnLeft();          break; // Left
    case 6: forwardRight();      break; // Forward + Right
    case 7: forwardLeft();       break; // Forward + Left
    case 8: reverseRight();      break; // Reverse + Right
    case 9: reverseLeft();       break; // Reverse + Left
  }
}

// Motor Control Functions (Customize for your hardware)
void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
}

void moveForward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
}

void moveReverse() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}


void turnRight() {
  digitalWrite(motorA1, HIGH); // Right motor forward
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);  // Left motor stopped
  digitalWrite(motorB2, LOW);
}

void turnLeft() {
  digitalWrite(motorA1, LOW);  // Right motor stopped
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH); // Left motor forward
  digitalWrite(motorB2, LOW);
}

// Add functions for forwardRight(), forwardLeft(), etc., as needed.