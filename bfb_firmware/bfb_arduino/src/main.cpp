#include <Arduino.h>
#include <Servo.h>

Servo servo1;
Servo servo2;

int servo1Neutral = 75;
int servo2Neutral = 100;

int lastServo1Angle = 0;
int lastServo2Angle = 0;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);  // Connect servo 1 to pin 9
  servo2.attach(10); // Connect servo 2 to pin 10

  // Initialize servos to neutral position
  servo1.write(servo1Neutral);
  servo2.write(servo2Neutral);

  // Update last angles
  lastServo1Angle = servo1Neutral;
  lastServo2Angle = servo2Neutral;
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the command

    if (command == "s0") {
      servo1.write(servo1Neutral); // Set servo 1 to neutral position
      lastServo1Angle = servo1Neutral; // Update last angle

      servo2.write(servo2Neutral); // Set servo 2 to neutral position
      lastServo2Angle = servo2Neutral; // Update last angle
    }
    if (command == "s1") {
      servo1.write(lastServo1Angle + 1); // Increment servo 1 angle by +1
      lastServo1Angle += 1; // Update last angle
    }
    if (command == "s2") {
      servo1.write(lastServo2Angle - 1); // Increment servo 1 angle by -1
      lastServo2Angle -= 1; // Update last angle
    }
    if (command == "s3") {
      servo2.write(lastServo2Angle + 1); // Increment servo 2 angle by +1
      lastServo2Angle += 1; // Update last angle
    }
    if (command == "s4") {
      servo2.write(lastServo2Angle - 1); // Increment servo 2 angle by -1
      lastServo2Angle -= 1; // Update last angle
    }
  }
}