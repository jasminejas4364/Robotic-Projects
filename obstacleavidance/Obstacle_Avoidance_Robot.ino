#include <NewPing.h>

#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5

#define echo A0
#define trigger A1

const int MAX_DISTANCE = 200;
const int TURN_DELAY = 400;   // Delay for turning 90 degrees
const int Set = 20;           // Distance threshold

int distance_F, distance_L, distance_R;

NewPing sonar(trigger, echo, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);

  analogWrite(enA, 100);  
  analogWrite(enB, 127);  

}

void loop() {
  distance_F = readPing();
  Serial.print("D F=");
  Serial.println(distance_F);

  if (distance_F <= Set) {
    stopMotors();
    delay(300);
    moveBackward();
    delay(400);
    stopMotors();
    delay(300);

    distance_L = lookLeft();
    delay(300);
    distance_R = lookRight();
    delay(300);

    if (distance_R > distance_L) {
      turnRight();
      delay(TURN_DELAY);
    } else {
      turnLeft();
      delay(TURN_DELAY);
    }
  } else {
    moveForward();
  }
}

int lookRight() {
  turnRight();
  delay(TURN_DELAY);
  stopMotors();
  int distance = readPing();
  turnLeft();  // Return to original position
  delay(TURN_DELAY);
  stopMotors();
  return distance;
}

int lookLeft() {
  turnLeft();
  delay(TURN_DELAY);
  stopMotors();
  int distance = readPing();
  turnRight();  // Return to original position
  delay(TURN_DELAY);
  stopMotors();
  return distance;
}

int readPing() {
  int cm = sonar.ping_cm();
  return (cm == 0) ? MAX_DISTANCE : cm;
}

void moveForward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveBackward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
