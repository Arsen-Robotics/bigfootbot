#include <Arduino.h>
#include <Servo.h>

// --- Pin definitions ---
#define buzzerPin 52
#define headLightPin 53
#define beaconLightPin 51

// L298N #1
#define linActSpeedPin 2
#define linActMove1Pin 3
#define linActMove2Pin 4

Servo tiltServo;
Servo panServo;

// --- Variables ---
int linActSpeed = 255;

// Neutral position for servos in degrees
int tiltNeutral = 75;
int panNeutral = 72;

int lastTiltAngle = 0;
int lastPanAngle = 0;

// Angle for quick look to left/right
int lookAngle = 60;
// bool lookToLeftSide = false;
// bool lookToRightSide = false;

// unsigned long lastCommandTime = 0;  // Last time a command was received
// unsigned long returnDelay = 100;   // Time (in ms) to wait after qucick look button was released, before returning to neutral position

// Beacon light
bool beaconBlinking = false;
unsigned long previousBeaconMillis = 0;
const long blinkIntervalOn = 500;  // 0.5 second on
const long blinkIntervalOff = 500;  // 0.5 second off
bool beaconState = false;

// Reverse beeper
bool reverseBeeping = false;
unsigned long previousBeepMillis = 0;
const long beepIntervalOn = 500;  // 0.5 second on
const long beepIntervalOff = 250;  // 1/4 second off
bool beepState = false;

void moveLinAct(int speed, int move1, int move2) {
  analogWrite(linActSpeedPin, speed);
  digitalWrite(linActMove1Pin, move1);
  digitalWrite(linActMove2Pin, move2);
}

void setup() {
  Serial.begin(9600);
  // tiltServo.attach(9);  // Connect servo 1 to pin 9
  // panServo.attach(10); // Connect servo 2 to pin 10

  // Initialize servos to neutral position
  // tiltServo.write(tiltNeutral);
  // panServo.write(panNeutral);

  // Update last angles
  // lastPanAngle = panNeutral;
  // lastTiltAngle = tiltNeutral;

  pinMode(buzzerPin, OUTPUT);
  pinMode(headLightPin, OUTPUT);
  pinMode(beaconLightPin, OUTPUT);

  pinMode(linActSpeedPin, OUTPUT);
  pinMode(linActMove1Pin, OUTPUT);
  pinMode(linActMove2Pin, OUTPUT);

  digitalWrite(buzzerPin, HIGH);
  digitalWrite(headLightPin, HIGH);
  digitalWrite(beaconLightPin, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read the command from serial

    // Reset camera tilt to neutral position
    if (command == "0") {
      tiltServo.write(tiltNeutral); // Set servo 1 to neutral position
      lastTiltAngle = tiltNeutral; // Update last angle
    }

    // Reset camera pan to neutral position
    if (command == "11") {
      panServo.write(panNeutral); // Set servo 2 to neutral position
      lastPanAngle = panNeutral; // Update last angle
    }

    // Tilt camera up
    if (command == "1") {
      lastTiltAngle = constrain(lastTiltAngle + 2, 0, 180); // Ensure angle stays between 0 and 180
      tiltServo.write(lastTiltAngle); // Increment servo 1 angle by +2 deg
    }

    // Tilt camera down
    if (command == "2") {
      lastTiltAngle = constrain(lastTiltAngle - 2, 0, 180); // Ensure angle stays between 0 and 180
      tiltServo.write(lastTiltAngle); // Decrement servo 1 angle by -2 deg
    }

    // Quick look to the left
    if (command == "3") {
      // lastCommandTime = millis();  // Update last command time

      // if (lookToLeftSide == false || lookToRightSide == true) {
      // lookToLeftSide = true;
      // lookToRightSide = false;
      panServo.write(panNeutral + lookAngle); // Look to the left
      lastPanAngle = panNeutral + lookAngle;
      // }
    }

    // Quick look to the right
    if (command == "4") {
      // lastCommandTime = millis();  // Update last command time

      // if (lookToRightSide == false || lookToLeftSide == true) {
      // lookToRightSide = true;
      // lookToLeftSide = false;
      panServo.write(panNeutral - lookAngle); // Look to the right
      lastPanAngle = panNeutral - lookAngle;
      // }
    }

    // Pan camera to the left
    if (command == "5") {
      lastPanAngle = constrain(lastPanAngle + 3, 0, 180); // Ensure angle stays between 0 and 180
      panServo.write(lastPanAngle); // Increment servo 1 angle by +3 deg
    }

    // Pan camera to the right
    if (command == "6") {
      lastPanAngle = constrain(lastPanAngle - 3, 0, 180); // Ensure angle stays between 0 and 180
      panServo.write(lastPanAngle); // Decrement servo 1 angle by -3 deg
    }

    // Buzzer on
    if (command == "7") {
      digitalWrite(buzzerPin, LOW);
    }

    // Buzzer off
    if (command == "8") {
      digitalWrite(buzzerPin, HIGH);
    }

    // Light off
    if (command == "9") {
      digitalWrite(headLightPin, HIGH);
    }

    // Light on
    if (command == "10") {
      digitalWrite(headLightPin, LOW);
    }

    // Linear actuator up
    if (command == "12") {
      moveLinAct(linActSpeed, HIGH, LOW);
    }

    // Linear actuator down
    if (command == "13") {
      moveLinAct(linActSpeed, LOW, HIGH);
    }

    // Linear actuator stop
    if (command == "14") {
      moveLinAct(0, LOW, LOW);
    }

    // Beacon light start blinking
    if (command == "15") {
      beaconBlinking = true;
    }

    // Beacon light off
    if (command == "16") {
      beaconBlinking = false;
      digitalWrite(beaconLightPin, LOW);  // Turn off beacon
    }

    // Reverse beeper start beeping
    if (command == "17") {
      reverseBeeping = true;
    }

    // Reverse beeper off
    if (command == "18") {
      reverseBeeping = false;
      digitalWrite(buzzerPin, HIGH);  // Turn off beeper
    }
  }

  // Handle beacon blinking
  if (beaconBlinking) {
    unsigned long currentMillis = millis();
    if (beaconState && currentMillis - previousBeaconMillis >= blinkIntervalOn) {
      beaconState = false;
      previousBeaconMillis = currentMillis;
      digitalWrite(beaconLightPin, LOW);  // Turn off beacon
    }
    else if (!beaconState && currentMillis - previousBeaconMillis >= blinkIntervalOff) {
      beaconState = true;
      previousBeaconMillis = currentMillis;
      digitalWrite(beaconLightPin, HIGH);  // Turn on beacon
    }
  }
  // Handle reverse beeping
  if (reverseBeeping) {
    unsigned long currentMillis = millis();
    if (beepState && currentMillis - previousBeepMillis >= beepIntervalOn) {
      beepState = false;
      previousBeepMillis = currentMillis;
      digitalWrite(buzzerPin, HIGH);  // Turn off beeper
    }
    else if (!beepState && currentMillis - previousBeepMillis >= beepIntervalOff) {
      beepState = true;
      previousBeepMillis = currentMillis;
      digitalWrite(buzzerPin, LOW);  // Turn on beeper
    }
  }
  // If no quick look command was received for a while, return to neutral position
  // if ((millis() - lastCommandTime > returnDelay) && (lookToLeftSide || lookToRightSide)) {
  //   panServo.write(panNeutral);  // Move pan back to neutral
  //   lastPanAngle = panNeutral;
  //   lookToLeftSide = false;
  //   lookToRightSide = false;

  //   //delay(200);  // Wait for the servo to reach the neutral position
  //   // panServo.detach();  // Detach the servo to stop it from auto adjusting
  // }
}