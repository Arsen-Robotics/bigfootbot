#include <Arduino.h>
//#include "QuickMedianLib.h"

// Motors control
#define RPWM_1 8
#define LPWM_1 9

#define RPWM_2 10
#define LPWM_2 11

// Left and right motor identifiers
#define LEFT 1
#define RIGHT 2

// Auto-stop
#define AUTO_STOP_INTERVAL 200
long lastMotorCommand = AUTO_STOP_INTERVAL;

// Relays control
#define RELAY_1 22
#define RELAY_2 23

//Distance in cm when ultrasonic sensor starts sending distance over serial
//#define DANGER_DISTANCE 100

//#define DISTANCES_ARRAY_SIZE 10

// A pair of varibles to help parse serial commands
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

// Array to hold ultrasonic sensors distances for median calculation
//int distances[DISTANCES_ARRAY_SIZE];

// Variable to hold the index of distances array element, which is going to be written next
//int distance_index;

//----------Distance Control-------------

//#define trig 36
//#define echo 37

// Returns distance in centimeters to an object from the ultrasonic sensor
/*int distanceToObs() {
  long duration;
  int distance;

  // Clears the trigPin
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echo, HIGH);

  // Calculating the distance
  distance = duration*0.034/2;

  //Serial.println("distance to obstacle: " + String(distance));

  return distance;
}*/

void relaysOn() {
  digitalWrite(RELAY_1, LOW);
  digitalWrite(RELAY_2, LOW);

  Serial.write(50); // 2
}

void relaysOff() {
  digitalWrite(RELAY_1, HIGH);
  digitalWrite(RELAY_2, HIGH);

  Serial.write(49); // 1
}

void motorLRot(boolean clockWise, int speed) {
  if (clockWise == true) {
    analogWrite(RPWM_1, speed);
    digitalWrite(LPWM_1, LOW);
  }
  else if (clockWise == false) {
    digitalWrite(RPWM_1, LOW);
    analogWrite(LPWM_1, speed);
  }
}

void motorRRot(boolean clockWise, int speed) {
  if (clockWise == true) {
    analogWrite(RPWM_2, speed);
    digitalWrite(LPWM_2, LOW);
  }
  else if (clockWise == false) {
    digitalWrite(RPWM_2, LOW);
    analogWrite(LPWM_2, speed);
  }
}

void stop() {
  digitalWrite(RPWM_1, LOW);
  digitalWrite(LPWM_1, LOW);

  digitalWrite(RPWM_2, LOW);
  digitalWrite(LPWM_2, LOW);
}

void setup() {
  Serial.begin(9600);

  //pinMode(trig, OUTPUT);
  //pinMode(echo, INPUT);

  //digitalWrite(trig, LOW);
  //delayMicroseconds(2);

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  
  digitalWrite(RELAY_1, HIGH);
  digitalWrite(RELAY_2, HIGH);

  pinMode(RPWM_1, OUTPUT);
  pinMode(LPWM_1, OUTPUT);

  pinMode(RPWM_2, OUTPUT);
  pinMode(LPWM_2, OUTPUT);
}

void setMotorSpeed(int motor, int speed) {
  int reverse = 0;

  if(speed < 0) {
    speed = -speed;
    reverse = 1;
  }
  if(speed > 255) {
    speed = 255;
  }

  if(motor == LEFT) {
    if(reverse == 0) {
      motorLRot(true, speed);
    }
    else if(reverse == 1) {
      motorLRot(false, speed);
    }
  }
  if(motor == RIGHT) {
    if(reverse == 0) {
      motorRRot(true, speed);
    }
    else if(reverse == 1){
      motorRRot(false, speed);
    }
  }
}

void setMotorSpeeds(int speed1, int speed2) {
  setMotorSpeed(LEFT, -speed1);
  setMotorSpeed(RIGHT, speed2);
}

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
    case 'm': // PWM motor control
      lastMotorCommand = millis();
      setMotorSpeeds(arg1, arg2);
      break;

    case 's': // Stop motors
      stop();
      break;

    case 'r': // Relays control
      if (arg1 == 2) {
        relaysOn();
      }
      if (arg1 == 1) {
        relaysOff();
      }
      break;
  }
}

void loop() {
  //if (distanceToObs() < DANGER_DISTANCE) {
  /*if (Serial.availableForWrite() > 0) {
    if (distance_index == DISTANCES_ARRAY_SIZE) {
      int median = QuickMedian<int>::GetMedian(distances, DISTANCES_ARRAY_SIZE);

      Serial.write(100); // d
      Serial.write(32); // space
      Serial.print(median);
      Serial.write(10); // carriage return (CR)

      distance_index = 0;
      delay(10);
    }
    if (distanceToObs() > 0) {
      distances[distance_index] = distanceToObs();
      distance_index += 1;
      delay(2);
    }
  }*/
  while (Serial.available() > 0) {   
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }

    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }

    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
      stop();
  }
}