#include <AFMotor.h>

// Motor initialization
AF_DCMotor motor1(1); // left front
AF_DCMotor motor2(2); // left rear
AF_DCMotor motor3(3); // right front
AF_DCMotor motor4(4); // right rear

// Pins for sensors
#define ECHO_PIN A0
#define TRIG_PIN A1
#define LEFT_IR_PIN A3
#define RIGHT_IR_PIN A2

// Threshold values for the ultrasonic sensor (in centimeters)
#define STOP_DISTANCE 35
#define REVERSE_DISTANCE 20
#define FORWARD_DISTANCE 150

// Base speed
int speed = 150;

// Function prototypes
void forward();
void reverse();
void stopMotors();
int measureDistance();

void setup() {
  // Pin setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
}

void loop() {
  int distance = measureDistance(); // Measure distance

  // Movement logic depending on the distance
  if (distance < REVERSE_DISTANCE) {
    reverse(); // Move backward
  } else if (distance > STOP_DISTANCE && distance <= FORWARD_DISTANCE) {
    forward(); // Move forward
  } else {
    stopMotors(); // Stop the motors
  }

  // Turning logic based on IR sensors
  int leftIR = analogRead(LEFT_IR_PIN);
  int rightIR = analogRead(RIGHT_IR_PIN);

  if (leftIR < 512) {
    turnRight(); // Turn right
  } else if (rightIR < 512) {
    turnLeft(); // Turn left
  }

  delay(50); // Minimal delay to prevent sensor debounce
}

// Function to measure distance using the ultrasonic sensor
int measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // Convert time to centimeters
}

// Motor control functions
void setMotorDirection(AF_DCMotor& motor, int direction) {
  if (direction == 1) {
    motor.run(FORWARD);
  } else if (direction == -1) {
    motor.run(BACKWARD);
  } else {
    motor.run(RELEASE);
  }
}

void forward() {
  motor1.setSpeed(speed);
  motor2.setSpeed(speed);
  motor3.setSpeed(speed);
  motor4.setSpeed(speed + 30);

  setMotorDirection(motor1, -1);  // Left front
  setMotorDirection(motor2, 1);   // Left rear
  setMotorDirection(motor3, -1);  // Right front
  setMotorDirection(motor4, -1);  // Right rear
}

void reverse() {
  motor1.setSpeed(speed + 30);
  motor2.setSpeed(speed + 30);
  motor3.setSpeed(speed + 30);
  motor4.setSpeed(speed + 30);

  setMotorDirection(motor1, 1);
  setMotorDirection(motor2, -1);
  setMotorDirection(motor3, 1);
  setMotorDirection(motor4, 1);
}

void stopMotors() {
  setMotorDirection(motor1, 0);
  setMotorDirection(motor2, 0);
  setMotorDirection(motor3, 0);
  setMotorDirection(motor4, 0);
}

void turnLeft() {
  motor1.setSpeed(speed + 70);
  motor2.setSpeed(0);  // Left wheel stops
  motor3.setSpeed(0);  // Right wheel stops
  motor4.setSpeed(speed + 70);

  setMotorDirection(motor1, -1);  // Left front motor backward
  setMotorDirection(motor2, -1);  // Left rear
  setMotorDirection(motor3, -1);  // Right front
  setMotorDirection(motor4, -1);  // Right rear

  delay(500); // Time to turn
}

void turnRight() {
  motor1.setSpeed(0);  // Left wheel stops
  motor2.setSpeed(speed + 70);
  motor3.setSpeed(speed + 70);
  motor4.setSpeed(0);  // Right wheel stops

  setMotorDirection(motor1, 1);  // Left front motor forward
  setMotorDirection(motor2, 1);  // Left rear motor forward
  setMotorDirection(motor3, -1); // Right front motor backward
  setMotorDirection(motor4, -1); // Right rear motor backward

  delay(500); // Time to turn
}
