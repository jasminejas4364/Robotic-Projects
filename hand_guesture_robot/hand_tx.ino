#include <Wire.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>

#define HC12_RX_PIN 3  // Connect to HC-12 TXD
#define HC12_TX_PIN 2  // Connect to HC-12 RXD

SoftwareSerial HC12(HC12_TX_PIN, HC12_RX_PIN);
MPU6050 mpu;

// Initial calibration offsets
float initialX = 0, initialY = 0, initialZ = 0;

// Thresholds for movement detection
const float forwardThreshold = 0.3;   // Example threshold for forward/backward
const float backwardThreshold = -0.3;
const float rightThreshold = 0.3;     // Example threshold for right/left
const float leftThreshold = -0.3;

void setup() {
  Serial.begin(9600);
  HC12.begin(9600);
Serial.println("MPU6050 initialize..");
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Halt if MPU6050 fails
  }

  Serial.println("Calibrating MPU6050...");
  delay(2000);
  calibrateInitialPosition();
  Serial.println("Calibration complete.");
  Serial.println("HC-12 Transmitter Ready!");
}

void loop() {
  // Read accelerometer values
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert to G values
  float xAcc = ax / 16384.0;
  float yAcc = ay / 16384.0;
  float zAcc = az / 16384.0;

  char command = detectMovement(xAcc, yAcc, zAcc);

  // Transmit command if movement detected
  if (command != 'n') { // 'n' represents no significant change
    HC12.println(command);
    Serial.println(String("Transmitted: ") + command);
  }

  delay(200); // Adjust as needed to reduce transmission frequency
}

void calibrateInitialPosition() {
  int samples = 100;
  float sumX = 0, sumY = 0, sumZ = 0;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    sumX += ax / 16384.0;
    sumY += ay / 16384.0;
    sumZ += az / 16384.0;

    delay(10);
  }

  initialX = sumX / samples;
  initialY = sumY / samples;
  initialZ = sumZ / samples;

  Serial.print("Initial X: "); Serial.println(initialX);
  Serial.print("Initial Y: "); Serial.println(initialY);
  Serial.print("Initial Z: "); Serial.println(initialZ);
}

char detectMovement(float xAcc, float yAcc, float zAcc) {
  float deltaX = xAcc - initialX;
  float deltaY = yAcc - initialY;
  float deltaZ = zAcc - initialZ;

  if (deltaX > forwardThreshold) return 'f';  // Forward
  if (deltaX < backwardThreshold) return 'b'; // Backward
  if (deltaY > rightThreshold) return 'l';    // Right
  if (deltaY < leftThreshold) return 'r';     // Left

  // Stop if near initial position
  if (abs(deltaX) < 1.0 && abs(deltaY) < 1.0) return 's';

  return 'n'; // No significant movement
}
