#define IR_SENSOR_RIGHT A1
#define IR_SENSOR_LEFT A0
#define MOTOR_SPEED 150

// Right motor
#define ENA 10
#define IN1 9
#define IN2 8

// Left motor
#define ENB 5
#define IN3 7
#define IN4 6

void setup() {
  Serial.begin(9600);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  
  rotateMotor(0, 0);  // Stop motors initially
}

void loop() {
  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  // Low - White (due to received IR) & High - Black (due to no received IR)
  // If none of the sensors detect the black line, go forward
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW) {
    rotateMotor(-MOTOR_SPEED, -MOTOR_SPEED);
  }
  // If right sensor detects black, turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW) {
    rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
  }
  // If left sensor detects black, turn left
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH) {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  }
  // If both sensors detect black, stop
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == HIGH) {
    rotateMotor(0, 0);
  }
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  // Control right motor
  if (rightMotorSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else if (rightMotorSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Control left motor
  if (leftMotorSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else if (leftMotorSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, abs(rightMotorSpeed));
  analogWrite(ENB, abs(leftMotorSpeed));
}
