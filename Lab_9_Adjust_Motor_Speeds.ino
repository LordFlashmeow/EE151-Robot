#include <Servo.h>

// Set servo pin
const int servoPin = 11;

Servo Servo1;

// Turning direction
// 0b00 is forwards
// 0b01 is turning right
// 0b10 is turning left
byte TurningDirection = 0b00;

// Set motor pins
const int motorAPin1 = 3; // Right Motor
const int motorAPin2 = 4;
const int motorBPin1 = 5; // Left Motor
const int motorBPin2 = 6;

// PWM motors pins to set speed of motors
const int motorAPWMPin = 2;
const int motorBPWMPin = 7;

// The desired speed for the robot to move at
int DesiredSpeed = 60;

// Steering gain to adjust how hard the turns can be
const int SteeringGain = 20;

// Pointing gain is how much we turn the echo sensor
const int PointingGain = 20;

// Light sensor threshold for identifying white and black
const int threshold = 100;

void setup() {

  Serial.begin(9600);

  // Attach and center servo
  Servo1.attach(servoPin);
  Servo1.write(90);


  // Set up the motor pins
  pinMode(motorAPin1, OUTPUT);
  pinMode(motorAPin2, OUTPUT);
  pinMode(motorBPin1, OUTPUT);
  pinMode(motorBPin2, OUTPUT);
  pinMode(motorAPWMPin, OUTPUT);
  pinMode(motorBPWMPin, OUTPUT);

  // Depower the motors
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, LOW);
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, LOW);
  analogWrite(motorAPWMPin, 0);
  analogWrite(motorBPWMPin, 0);


}

byte SensorCode = 0;
float PathError = 0;

void loop() {
  SensorCode = GetPathSensorStates();
  PathError = SensePathPositionError(SensorCode);


  if (PathError != 10) {  // If we get a non-bogus path error
    AdjustMotorSpeeds(PathError);
    UpdatePointingAngle(PathError);
  }

}

void Move(int LeftPWM, int RightPWM) {
  // Left motor forward
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  // Right motor forward
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);

  analogWrite(motorBPWMPin, LeftPWM);
  analogWrite(motorAPWMPin, RightPWM);

}

void AdjustMotorSpeeds(float PathError) {

  // Detect bogus value and maintain current turn
  if (PathError == 10) {
    return;
  }

  // SpeedAdjustmentPercent controls how hard the robot can turn
  int SpeedAdjustmentPercent = SteeringGain * PathError;

  int RightMotorSpeed = (100 - SpeedAdjustmentPercent) * DesiredSpeed / 100;
  int LeftMotorSpeed = (100 + SpeedAdjustmentPercent) * DesiredSpeed / 100;


  // Contstrain the motor speeds to acceptable values
  RightMotorSpeed = constrain(RightMotorSpeed, 0, 255);
  LeftMotorSpeed = constrain(LeftMotorSpeed, 0, 255);

  // Serial.print("  LeftMotorSpeed: ");
  // Serial.print(LeftMotorSpeed);
  // Serial.print("  RightMotorSpeed: ");
  // Serial.print(RightMotorSpeed);

  // If the robot is too far to the right, stop the left motor to turn tighter
  if (PathError <= -1.5) {
    LeftMotorSpeed = 0;
  }

  // If the robot is too far to the left, stop the right motor to turn tighter
  if (PathError >= 1.5) {
    RightMotorSpeed = 0;
  }


  Move(LeftMotorSpeed, RightMotorSpeed);
}

float SensePathPositionError(byte PathSensorStates) {
  // Calculates the distance between the robot and the line
  switch (PathSensorStates) {
    case 1:
      return 2;
    case 2:
      return 1;
    case 3:
      return 1.5;
    case 4:
      TurningDirection = 0;
      return 0;
    case 6:
      return 0.5;
    case 8:
      return -1;
    case 12:
      return -0.5;
    case 16:
      return -2.0;
    case 24:
      return -1.5;

    default:
      return 10;
  }
}

byte GetPathSensorStates() {
  // Reads the light sensors and returns a byte that represents which sensors see the black line
  byte code = 0b00000;

  bitWrite(code, 4, ReadLineSensor(A15, 43));
  bitWrite(code, 3, ReadLineSensor(A14, 45));
  bitWrite(code, 2, ReadLineSensor(A13, 47));
  bitWrite(code, 1, ReadLineSensor(A12, 49));
  bitWrite(code, 0, ReadLineSensor(A11, 51));

  // Serial.print("Sensor state: ");
  // Serial.println(code);

  return code;
}

int ReadLineSensor(int SensorAnalogInPin, int SensorDigitalOutPin) {
  // Reads a light sensor and returns true if it sees black
  int value = analogRead(SensorAnalogInPin);  // Read the brightness
  // Serial.print(value);
  // Serial.print("  ");
  if (value > threshold) {
    digitalWrite(SensorDigitalOutPin, HIGH);
    return 1;
  } else {
    digitalWrite(SensorDigitalOutPin, LOW);
    return 0;
  }
}

int UpdatePointingAngle(float PathError) {
  // if the error is a bogus value, use the previous servo value
  if (PathError == 10) {
    return Servo1.read();
  }

  int SensorPointingAngle = 90 + PointingGain * PathError;
  Servo1.write(SensorPointingAngle);
  return 90 + 10 * PathError;
}
