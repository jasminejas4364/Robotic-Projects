#include <Servo.h>

#define FLAME1 2
#define FLAME2 3
#define FLAME3 4

#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 8
#define SPEED 9

#define SERVO_PIN 10
#define RELAY 11

Servo fireServo;

void setup() {
    pinMode(FLAME1, INPUT);
    pinMode(FLAME2, INPUT);
    pinMode(FLAME3, INPUT);
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(SPEED, OUTPUT);

    pinMode(RELAY, OUTPUT);
    fireServo.attach(SERVO_PIN);

    digitalWrite(RELAY, LOW); // Pump OFF initially
    analogWrite(SPEED, 150);  // Set motor speed
    Serial.println("Welcome");

}

void loop() {
    int fire1 = digitalRead(FLAME1);
    int fire2 = digitalRead(FLAME2);
    int fire3 = digitalRead(FLAME3);

    if (fire1 == LOW || fire2 == LOW || fire3 == LOW) {
        stopMotors();  // Stop the robot

        if(fire1 == LOW) {
            fireServo.write(45); // Turn servo to face left
            delay(500);
        } 
        if (fire2 == LOW) {
            fireServo.write(90); // Face center
            delay(500);
        } 
        if (fire3 == LOW) {
            fireServo.write(135); // Turn servo to face right
            delay(500);
        }
        delay(500);
        digitalWrite(RELAY, HIGH); // Turn ON water pump
        delay(2000); // Keep spraying for 5 seconds
        digitalWrite(RELAY, LOW); // Turn OFF water pump
    } 
    else{
    moveForward(); // Resume movement
    }
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
