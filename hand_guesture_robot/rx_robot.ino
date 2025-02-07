#include <SoftwareSerial.h>
#define SPEED1 8
#define SPEED2 9
// Motor control pins for L293D
const int in1 = 4;
const int in2 = 5;
const int in3 = 6;
const int in4 = 7;

#define HC12_RX_PIN 2  // Connect to HC-12 TXD
#define HC12_TX_PIN 3  // Connect to HC-12 RXD

SoftwareSerial HC12(HC12_TX_PIN, HC12_RX_PIN);

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("HC-12 Receiver Initializing...");

  // Initialize HC-12 communication
  HC12.begin(9600);

  // Set motor control pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(SPEED1, OUTPUT);
  pinMode(SPEED2, OUTPUT);

  Serial.println("HC-12 Receiver Ready!");
  
  analogWrite(SPEED1, 255);
  analogWrite(SPEED2, 255);
}

// Function to stop all motors
void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Function to move forward
void moveForward() {

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Function to move backward
void moveBackward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
}

// Function to turn right
void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Function to turn left
void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void loop() {
  // Receive data from HC-12
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');  // Read until newline
    receivedData.trim();                                 // Remove leading/trailing whitespace

    if (receivedData.length() > 0) {
      Serial.println("Received: " + receivedData);

      // Execute motor control based on received command
      if (receivedData == "f") {
        moveForward();
        Serial.println("Moving Forward");
      } else if (receivedData == "b") {
        moveBackward();
        Serial.println("Moving Backward");
      } else if (receivedData == "r") {
        turnRight();
        Serial.println("Turning Right");
      } else if (receivedData == "l") {
        turnLeft();
        Serial.println("Turning Left");
      } else if (receivedData == "s") {
        stopMotors();
        Serial.println("Stopping");
      }
    }
  }
}
