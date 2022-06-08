/* Example sketch to control a stepper motor with TB6600 stepper motor driver, AccelStepper library and Arduino: acceleration and deceleration. More info: https://www.makerguides.com */

// Include the AccelStepper library:
#include <AccelStepper.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 4
#define motorInterfaceType 1
#define stepperMotorSignal 22

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(16000);
  stepper.setAcceleration(100);
  pinMode(stepperMotorSignal, OUTPUT);
  //Turn off power to stepper motor (LOW = on, HIGH = off)
  digitalWrite(stepperMotorSignal, HIGH);
  Serial.begin(9600);
  Serial.println("Set up finish, input steps.");
  Serial.println("Positive steps = rotate clockwise, negative steps = rotate counter clockewise.");
  
}

void loop() {
  if (Serial.available() > 0) {
    
    int incomingByte = Serial.parseInt();

    //Turn on power to stepper motor (LOW = on, HIGH = off)
    digitalWrite(stepperMotorSignal, LOW);

    // Set the target position(positive = cw, negative = ccw)
    stepper.moveTo(incomingByte);
    //Run to target position with set speed and acceleration/deceleration:
    stepper.runToPosition();

    Serial.println("Position reached");

    //Turn off power to stepper motor (LOW = on, HIGH = off)
    digitalWrite(stepperMotorSignal, HIGH);
  }

  else {
    delay(100);
  }
}
