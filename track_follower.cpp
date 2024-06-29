// Include necessary libraries
#include <QTRSensors.h>

// Define the number of sensors
#define NUM_SENSORS 8

// Create an object for the sensor array
QTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5, 6, 7}, NUM_SENSORS);

// Motor control pins
const int motorLeftForward = 3;
const int motorLeftBackward = 5;
const int motorRightForward = 6;
const int motorRightBackward = 9;

// PID constants
float Kp = 0.2;
float Ki = 0.0;
float Kd = 0.1;

// PID variables
float setPoint = 3500;
float input, output, error, previousError = 0;
float integral = 0;
float derivative = 0;

// Motor control function
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(motorLeftForward, leftSpeed);
    analogWrite(motorLeftBackward, 0);
  } else {
    analogWrite(motorLeftForward, 0);
    analogWrite(motorLeftBackward, -leftSpeed);
  }
  if (rightSpeed > 0) {
    analogWrite(motorRightForward, rightSpeed);
    analogWrite(motorRightBackward, 0);
  } else {
    analogWrite(motorRightForward, 0);
    analogWrite(motorRightBackward, -rightSpeed);
  }
}

void setup() {
  // Initialize motor control pins
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftBackward, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightBackward, OUTPUT);
  
  // Initialize sensor array
  qtra.calibrate();
}

void loop() {
  // Read the line position
  unsigned int sensorValues[NUM_SENSORS];
  int position = qtra.readLine(sensorValues);
  
  // Compute the error
  error = setPoint - position;
  
  // Compute PID terms
  integral += error;
  derivative = error - previousError;
  output = Kp * error + Ki * integral + Kd * derivative;
  
  // Update previous error
  previousError = error;
  
  // Set motor speeds
  int baseSpeed = 200;
  int leftSpeed = baseSpeed + output;
  int rightSpeed = baseSpeed - output;
  
  setMotorSpeed(leftSpeed, rightSpeed);
  
  // Add a small delay for stability
  delay(10);
}
