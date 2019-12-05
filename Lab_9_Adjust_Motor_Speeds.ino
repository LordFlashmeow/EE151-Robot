#include <Servo.h>

#include <TimerOne.h>


int ledState = LOW;
const int ledPin = 10;
const int buzzerPin = 50;


// Set servo pin
const int rangefinderServoPin = 11;

Servo rangefinderServo;

// Servo that controls the egg drop
const int eggServoPin = 9;
Servo eggServo;


// Pins used for rangefinder
const int RangeTriggerPin = 40;
const int RangeEchoPin = 41;

const unsigned long RangeTimeout = 4000;


// Set motor pins
const int motorAPin1 = 3; // Right Motor
const int motorAPin2 = 4;
const int motorBPin1 = 5; // Left Motor
const int motorBPin2 = 6;

// PWM motors pins to set speed of motors
const int motorAPWMPin = 2;
const int motorBPWMPin = 7;

// The desired speed for the robot to move at
const int DesiredSpeed = 60;

// Steering gain to adjust how hard the turns can be
const int SteeringGain = 20;

// Pointing gain is how much we turn the echo sensor
const int PointingGain = 20;

// Light sensor threshold for identifying white and black
const int threshold = 100;

void setup() {

  Serial.begin(57600);

  // Attach and center servo
  rangefinderServo.attach(rangefinderServoPin);
  rangefinderServo.write(90);

  eggServo.attach(eggServoPin);
  eggServo.write(0);

  // Rangefinder
  pinMode(RangeTriggerPin, OUTPUT);
  pinMode(RangeEchoPin, INPUT);

  // Initialize timer, which handles lights and beeper outside of the main thread
  Timer1.initialize(500000);   //Set the period to 500k microseconds, or 500 miliseconds
  Timer1.attachInterrupt(beepAndFlashLED);

  // Initialize LED and buzzer
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

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

  // delay 3 seconds to allow time to load egg
  delay(3000);

}

void loop() {
  // Obstacle detection
  if (measureDistance() < 4) {
    StopMotors();

    // make a noise if there is an obstacle in the way
    // Repeat check for obstacle every 100 ms
    tone(buzzerPin, 600);
    delay(100);

    // Restart the loop to check again
    return;
  }

  StartMotors();
  FollowLine();

}

void FollowLine() {
  byte SensorCode = GetPathSensorStates();
  float PathError = SensePathPositionError(SensorCode);


  if (PathError != 10) {  // If we get a non-bogus path error, change motor speeds
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

  // Detect bogus value and maintain current motor speeds
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

    case 31:  // All the sensors are reading a black line
      DropEgg();

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

  return code;
}

int ReadLineSensor(int SensorAnalogInPin, int SensorDigitalOutPin) {
  // Reads a light sensor and returns true if it sees black
  int value = analogRead(SensorAnalogInPin);  // Read the brightness

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
    return rangefinderServo.read();
  }

  int SensorPointingAngle = 90 + PointingGain * PathError;
  rangefinderServo.write(SensorPointingAngle);
  return 90 + 10 * PathError;
}

void StopMotors() {
  // Left motor
  digitalWrite(motorAPin1, HIGH);
  digitalWrite(motorAPin2, HIGH);
  // Right motor
  digitalWrite(motorBPin1, HIGH);
  digitalWrite(motorBPin2, HIGH);
}

void StartMotors() {
  // Left motor
  digitalWrite(motorAPin1, LOW);
  digitalWrite(motorAPin2, HIGH);
  // Right motor
  digitalWrite(motorBPin1, LOW);
  digitalWrite(motorBPin2, HIGH);
}

void DropEgg() {
  // Keep advancing until 2.7 in away
  while (measureDistance() > 2.7) {
    FollowLine();
  }

  // Stop the motors
  StopMotors();

  // Stop the led and beeping
  Timer1.detachInterrupt();

  // Wait for robot to come to a stop before dropping egg
  delay(250);
  eggServo.write(75);

  // run setup for music file
  musicSetup();

  // play music forever until turned off
  while (true) {
    playMusic();
  }
}

float measureDistance() {
  // Activate the distance sensor
  digitalWrite(RangeTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(RangeTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(RangeTriggerPin, LOW);

  unsigned long EchoDelay = pulseIn(RangeEchoPin, HIGH, RangeTimeout);

  if (EchoDelay == 0.0) {   // rangefinder hit timeout, assume a large distance
    return 50;
  }

  // if the object is actually less than 2 inches away, the scanner may give bad results
  if (EchoDelay < 300) {
    return 0;
  }

  float Distance = (EchoDelay / 74.0) / 2.0;

  return Distance;
}

void beepAndFlashLED() {
  // either turn led on or off, depending on state
  digitalWrite(ledPin, ledState);

  // if the ledstate is high, turn on the buzzer, otherwise turn it off
  // creates a beeping effect
  if (ledState) {
    tone(buzzerPin, 400);
  } else {
    noTone(buzzerPin);
  }

  ledState = !ledState; // Invert the state

}
