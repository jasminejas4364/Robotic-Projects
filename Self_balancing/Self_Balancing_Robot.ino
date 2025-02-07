#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor driver pins
#define ENA 3   // PWM control for left motor
#define IN1 5   // Left motor direction
#define IN2 6
#define ENB 11  // PWM control for right motor
#define IN3 9   // Right motor direction
#define IN4 10

// PID variables
float Kp = 5.0;  // Proportional gain
float Ki = 0.1;   // Integral gain
float Kd = 5.0;  // Derivative gain

float setpoint = 0;  // Target angle (balanced position)
float angle, error, prevError = 0, integral = 0, derivative;
int motorSpeed;

// Angle calculation using accelerometer and gyroscope (complementary filter)
float accelAngle, gyroRate, filteredAngle;
unsigned long previousTime = 0;
float dt = 0.01;  // Loop time in seconds (10 ms)

long ax_offset = 0, az_offset = 0;
float initialAngle = 0;

// Function to control motor speed and direction
void setMotorSpeed(int speed) {
  int absSpeed = abs(speed);
  absSpeed = constrain(absSpeed, 0, 160);

  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  analogWrite(ENA, absSpeed);
  analogWrite(ENB, absSpeed);
}

// Function to calibrate the MPU6050 sensor
void calibrateMPU() {
  long axSum = 0, azSum = 0;
  int samples = 100;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    axSum += ax;
    azSum += az;
    delay(5);
  }

  ax_offset = axSum / samples;
  az_offset = azSum / samples;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1)
      ;
  }
  Serial.println("MPU6050 initialized.");

  // Calibrate the sensor
  calibrateMPU();

  // Capture the initial tilt as baseline
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  initialAngle = atan2(ax - ax_offset, az - az_offset) * 180 / PI;
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate tilt angle using accelerometer
  accelAngle = atan2(ax - ax_offset, az - az_offset) * 180 / PI;

  // Calculate angular velocity using gyroscope
  gyroRate = gy / 131.0;  // Convert to degrees per second

  // Implement complementary filter for better angle estimation
  unsigned long currentTime = micros();
  dt = (currentTime - previousTime) / 1000000.0;  // Time in seconds
  previousTime = currentTime;

  filteredAngle = 0.98 * (filteredAngle + gyroRate * dt) + 0.02 * accelAngle;

  // PID calculations
  error = setpoint - filteredAngle;
  integral += error * dt;
  integral = constrain(integral, -50, 50);  // Clamp integral to prevent windup
  derivative = (error - prevError) / dt;
  motorSpeed = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Add deadband for small errors
  if (abs(error) < 2) {
    motorSpeed = 0;  // Stop motor for small deviations
  }

  motorSpeed = constrain(motorSpeed, -160, 160);
  setMotorSpeed(motorSpeed);

  prevError = error;

  // Debugging output
  Serial.print("Angle: ");
  Serial.print(filteredAngle);
  Serial.print(" | Speed: ");
  Serial.println(motorSpeed);

  delay(2);  // Small delay for smoother control
}